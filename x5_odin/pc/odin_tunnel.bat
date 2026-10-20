@echo off
rem 端口转发：把 X5 上的 8080（Odin1 网页预览）映射到本机 127.0.0.1:8080
rem 用法：odin_tunnel.bat [X5_IP]      默认 172.17.90.239
rem 结束：Ctrl-C 或关掉窗口
setlocal
set "PLINK=%~dp0..\..\_downloads\putty\plink.exe"
set "X5=%~1"
if "%X5%"=="" set "X5=172.17.90.239"
set "HOSTKEY=SHA256:K8oyEoddaZ4keNu39exAqMJLDEcncgoQGcRPQ898Bb4"

echo [odin] 127.0.0.1:8080  ->  %X5%:8080
echo [odin] 浏览器打开 http://127.0.0.1:8080/
"%PLINK%" -batch -hostkey %HOSTKEY% -pw sunrise ^
  -L 8080:127.0.0.1:8080 -N sunrise@%X5%

