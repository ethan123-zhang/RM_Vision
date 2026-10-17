"""Minimal PDF text extractor: stdlib only (zlib + re)."""
import re
import sys
import zlib


def load_objects(data):
    objs = {}
    for m in re.finditer(rb"(\d+)\s+(\d+)\s+obj\b", data):
        num = int(m.group(1))
        start = m.end()
        end = data.find(b"endobj", start)
        if end == -1:
            continue
        objs[num] = data[start:end]
    return objs


def get_stream(body):
    m = re.search(rb"stream\r\n|stream\n|stream\r", body)
    if not m:
        return None
    raw = body[m.end():]
    e = raw.rfind(b"endstream")
    if e != -1:
        raw = raw[:e]
    dict_part = body[:m.start()]
    if b"FlateDecode" in dict_part:
        for trim in (0, 1, 2):
            try:
                return zlib.decompress(raw[trim:] if trim else raw)
            except zlib.error:
                continue
        try:
            d = zlib.decompressobj()
            return d.decompress(raw)
        except zlib.error:
            return None
    return raw


def is_objstm(body):
    return b"/ObjStm" in body


def objstm_params(dict_part):
    """Read /N and /First from the ObjStm DICTIONARY (not the decoded data)."""
    hm = re.search(rb"/First\s+(\d+)", dict_part)
    first = int(hm.group(1)) if hm else 0
    nm = re.search(rb"/N\s+(\d+)", dict_part)
    n = int(nm.group(1)) if nm else 0
    return n, first


def parse_objstm(dec, dict_part=None, n=None, first=None):
    """Object stream layout: N pairs 'objnum offset' (whitespace separated),
    then /First bytes in, the concatenated object bodies. Offsets are relative
    to /First."""
    if dict_part is not None:
        n, first = objstm_params(dict_part)
    if n is None:
        nm = re.search(rb"/N\s+(\d+)", dec)
        n = int(nm.group(1)) if nm else 0
    if first is None:
        hm = re.search(rb"/First\s+(\d+)", dec)
        first = int(hm.group(1)) if hm else 0

    header = dec[:first]
    toks = header.split()
    ints = []
    for t in toks:
        if re.fullmatch(rb"\d+", t):
            ints.append(int(t))
        else:
            break
    if n:
        ints = ints[:2 * n]
    if len(ints) < 2:
        return []

    out = []
    for i in range(0, len(ints) - 1, 2):
        num = ints[i]
        off = ints[i + 1]
        start = first + off
        if i + 2 < len(ints):
            end = first + ints[i + 2]
        else:
            end = len(dec)
        if start >= len(dec):
            continue
        out.append((num, dec[start:end]))
    return out


def parse_tounicode(cm):
    mp = {}

    def to_str(h):
        h = re.sub(rb"[^0-9A-Fa-f]", b"", h)
        if len(h) % 4:
            h += b"0" * (4 - len(h) % 4)
        try:
            return bytes.fromhex(h.decode()).decode("utf-16-be", "ignore")
        except Exception:
            return ""

    for blk in re.findall(rb"beginbfchar(.*?)endbfchar", cm, re.S):
        for s, d in re.findall(rb"<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>", blk):
            mp[int(s, 16)] = to_str(d)
    for blk in re.findall(rb"beginbfrange(.*?)endbfrange", cm, re.S):
        for m in re.finditer(
                rb"<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>\s*(?:<([0-9A-Fa-f]+)>|\[(.*?)\])",
                blk, re.S):
            lo, hi = int(m.group(1), 16), int(m.group(2), 16)
            if m.group(3):
                base = int(m.group(3), 16)
                w = len(m.group(3)) // 4
                for i in range(hi - lo + 1):
                    mp[lo + i] = to_str(format(base + i, "0%dx" % (w * 4)).encode())
            elif m.group(4):
                for i, it in enumerate(re.findall(rb"<([0-9A-Fa-f]+)>", m.group(4))):
                    if lo + i <= hi:
                        mp[lo + i] = to_str(it)
    return mp


def cmap_for_font(body, objs, cache):
    m = re.search(rb"/ToUnicode\s+(\d+)\s+\d+\s+R", body)
    if not m:
        return {}
    num = int(m.group(1))
    if num in cache:
        return cache[num]
    t = objs.get(num)
    if not t:
        return {}
    dec = get_stream(t)
    if not dec:
        return {}
    mp = parse_tounicode(dec)
    cache[num] = mp
    return mp


def decode_lit(s):
    out = bytearray()
    i = 0
    simple = {0x6E: 10, 0x72: 13, 0x74: 9, 0x62: 8, 0x66: 12, 0x28: 40, 0x29: 41, 0x5C: 92}
    while i < len(s):
        c = s[i]
        if c == 0x5C and i + 1 < len(s):
            n = s[i + 1]
            if n in simple:
                out.append(simple[n]); i += 2; continue
            if 0x30 <= n <= 0x37:
                j = i + 1
                o = b""
                while j < len(s) and len(o) < 3 and 0x30 <= s[j] <= 0x37:
                    o += bytes([s[j]]); j += 1
                out.append(int(o, 8) & 0xFF); i = j; continue
            i += 2; continue
        out.append(c); i += 1
    return bytes(out)


def render(tok, cmap):
    kind, val = tok
    if kind == "hex":
        h = re.sub(rb"[^0-9A-Fa-f]", b"", val)
        if len(h) % 2:
            h += b"0"
        raw = bytes.fromhex(h.decode())
    else:
        raw = decode_lit(val)
    if cmap:
        wide = max(cmap) > 0xFF
        if wide:
            return "".join(cmap.get((raw[i] << 8) | raw[i + 1], "")
                           for i in range(0, len(raw) - 1, 2))
        return "".join(cmap.get(b, "") for b in raw)
    if len(raw) >= 2 and raw[0] == 0xFE and raw[1] == 0xFF:
        return raw[2:].decode("utf-16-be", "ignore")
    if raw and raw.count(0) > len(raw) // 3 and len(raw) % 2 == 0:
        return raw.decode("utf-16-be", "ignore")
    return raw.decode("latin-1", "ignore")


def tokenize(content):
    toks = []
    i, n = 0, len(content)
    while i < n:
        c = content[i]
        if c in b" \r\n\t\f\x00":
            i += 1; continue
        if c == 0x25:
            j = content.find(b"\n", i)
            i = n if j == -1 else j + 1; continue
        if c == 0x28:
            depth = 1; j = i + 1
            while j < n and depth:
                if content[j] == 0x5C:
                    j += 2; continue
                if content[j] == 0x28:
                    depth += 1
                elif content[j] == 0x29:
                    depth -= 1
                j += 1
            if j > n:
                j = n
            toks.append(("str", content[i + 1:min(j, n)])); i = max(j, i + 1); continue
        if c == 0x3C and i + 1 < n and content[i + 1] != 0x3C:
            j = content.find(b">", i)
            if j == -1:
                break
            toks.append(("hex", content[i + 1:j])); i = j + 1; continue
        if c == 0x3C and i + 1 < n and content[i + 1] == 0x3C:
            toks.append(("op", b"<<")); i += 2; continue
        if c in (0x5B, 0x5D):
            toks.append(("op", bytes([c]))); i += 1; continue
        if c == 0x2F:
            j = i + 1
            while j < n and content[j] not in b" \r\n\t\f\x00/[]<>()%":
                j += 1
            toks.append(("name", content[i + 1:j])); i = j; continue
        j = i
        while j < n and content[j] not in b" \r\n\t\f\x00/[]<>()%":
            j += 1
        if j == i:
            # unmatched delimiter byte: skip it so the loop always advances
            i += 1
            continue
        word = content[i:j]
        # A bare word that is not a number is an OPERATOR (Tf, Tj, ET, BDC, ...).
        try:
            float(word)
            toks.append(("num", word))
        except ValueError:
            toks.append(("op", word))
        i = j; continue
    return toks


def text_from_content(content, fonts):
    toks = tokenize(content)
    out = []
    cur = None
    for idx, (kind, val) in enumerate(toks):
        if kind != "op":
            continue
        if val == b"Tf":
            for k in range(idx - 1, -1, -1):
                if toks[k][0] == "name":
                    cur = fonts.get(toks[k][1].decode("latin-1"))
                    break
        elif val in (b"Tj", b"'", b'"'):
            for k in range(idx - 1, -1, -1):
                if toks[k][0] in ("str", "hex"):
                    out.append(render(toks[k], cur)); break
            if val in (b"'", b'"'):
                out.append("\n")
        elif val == b"TJ":
            items = []
            for k in range(idx - 1, -1, -1):
                if toks[k][0] == "op" and toks[k][1] == b"[":
                    break
                if toks[k][0] in ("str", "hex", "num"):
                    items.append(toks[k])
            items.reverse()
            buf = []
            for it in items:
                if it[0] == "num":
                    try:
                        if float(it[1]) < -180:
                            buf.append(" ")
                    except ValueError:
                        pass
                else:
                    buf.append(render(it, cur))
            out.append("".join(buf))
        elif val in (b"Td", b"TD", b"T*"):
            out.append("\n")
        elif val == b"ET":
            out.append("\n")
    return "".join(out)


def font_map(page_body, objs, cache):
    fonts = {}
    res = page_body
    m = re.search(rb"/Resources\s+(\d+)\s+\d+\s+R", page_body)
    if m:
        res = objs.get(int(m.group(1)), page_body)
    fm = re.search(rb"/Font\s*<<(.*?)>>", res, re.S)
    if not fm:
        m2 = re.search(rb"/Font\s+(\d+)\s+\d+\s+R", res)
        if m2:
            fobj = objs.get(int(m2.group(1)), b"")
            fm = re.search(rb"<<(.*?)>>", fobj, re.S)
    if not fm:
        return fonts
    for name, num in re.findall(rb"/([A-Za-z0-9#_.\-]+)\s+(\d+)\s+\d+\s+R", fm.group(1)):
        fb = objs.get(int(num))
        if fb is not None:
            fonts[name.decode("latin-1")] = cmap_for_font(fb, objs, cache)
    return fonts


def extract(path):
    data = open(path, "rb").read()
    objs = load_objects(data)
    extra = {}
    for num, body in list(objs.items()):
        if is_objstm(body):
            dec = get_stream(body)
            if dec:
                di = body.find(b"stream")
                dict_part = body[:di] if di != -1 else body
                for on, ob in parse_objstm(dec, dict_part):
                    extra.setdefault(on, ob)
    objs.update(extra)

    cache = {}
    order = []

    def walk(num, seen):
        if num in seen:
            return
        seen.add(num)
        body = objs.get(num)
        if body is None:
            return
        if re.search(rb"/Type\s*/Page(?![sA-Za-z])", body):
            order.append(num); return
        km = re.search(rb"/Kids\s*\[(.*?)\]", body, re.S)
        if km:
            for k in re.findall(rb"(\d+)\s+\d+\s+R", km.group(1)):
                walk(int(k), seen)

    rm = re.search(rb"/Root\s+(\d+)\s+\d+\s+R", data)
    if rm:
        walk(int(rm.group(1)), set())

    pages = [(n, objs[n]) for n in order if n in objs]
    if not pages:
        pages = [(n, b) for n, b in objs.items()
                 if re.search(rb"/Type\s*/Page(?![sA-Za-z])", b)]
        pages.sort(key=lambda it: (int(re.search(rb"/StructParents\s+(\d+)", it[1]).group(1))
                                   if re.search(rb"/StructParents\s+(\d+)", it[1]) else it[0]))

    chunks = []
    for num, body in pages:
        contents = b""
        cm = re.search(rb"/Contents\s+(\d+)\s+\d+\s+R", body)
        if cm:
            contents = get_stream(objs.get(int(cm.group(1)), b"")) or b""
        else:
            cm2 = re.search(rb"/Contents\s*\[(.*?)\]", body, re.S)
            if cm2:
                parts = [get_stream(objs.get(int(r), b"")) or b""
                         for r in re.findall(rb"(\d+)\s+\d+\s+R", cm2.group(1))]
                contents = b"\n".join(parts)
        if not contents:
            continue
        txt = text_from_content(contents, font_map(body, objs, cache))
        chunks.append(re.sub(r"\n{3,}", "\n\n", txt))
    return chunks


if __name__ == "__main__":
    pages = extract(sys.argv[1])
    with open(sys.argv[2], "w", encoding="utf-8", errors="replace") as f:
        for i, p in enumerate(pages, 1):
            f.write("\n\n===== PAGE %d =====\n" % i)
            f.write(p)
    print("pages=%d chars=%d -> %s" % (len(pages), sum(len(p) for p in pages), sys.argv[2]))
