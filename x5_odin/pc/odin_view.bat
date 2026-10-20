@echo off
rem 一键：后台开隧道 + 打开浏览器看 Odin1 实时画面
rem 用法：odin_view.bat [X5_IP]        默认 172.17.90.239
setlocal
set "X5=%~1"
if "%X5%"=="" set "X5=172.17.90.239"

start "odin-tunnel" /min cmd /c ""%~dp0odin_tunnel.bat" %X5%"
timeout /t 4 /nobreak >nul
start "" "http://127.0.0.1:8080/"
