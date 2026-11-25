# Prueba la estación web en este PC, sin cuenta de Cloudflare:
#
#   powershell -ExecutionPolicy Bypass -File estacion_web\probar_local.ps1
#   powershell -ExecutionPolicy Bypass -File estacion_web\probar_local.ps1 -Correo tu.correo@javerianacali.edu.co
#
# Crea una base local (en la carpeta temporal, fuera de OneDrive) con las credenciales de include/wearme_credentials.h,
# simula el inicio de sesión con ese correo (como administrador) y abre la estación en http://localhost:8787.
# Ctrl+C para terminar: borra el archivo de sesión simulada (.dev.vars). La base local queda para la próxima vez;
# -Limpiar la borra y la vuelve a crear.
param(
  [string]$Correo = "admin@local",
  [switch]$Limpiar
)
# "Continue": npx escribe avisos por stderr ("npm notice") y Windows PowerShell 5.1 los tomaría como error fatal
$ErrorActionPreference = "Continue"
function Fallo($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }
$Web = $PSScriptRoot
$Root = Split-Path $Web -Parent
$Estado = Join-Path $env:TEMP "wearme_estacion_local"
$Wrangler = "wrangler@4.156.0"
Set-Location $Web

if ($Limpiar -and (Test-Path $Estado)) { Remove-Item -Recurse -Force $Estado }

# 1. Página y firmwares (si no están generados)
if (-not (Test-Path (Join-Path $Web "public\firmware\firmwares.json"))) {
  Write-Host "Generando la página y los firmwares..."
  python (Join-Path $Root "tools\build_estacion.py") --web --no-build
  if ($LASTEXITCODE -ne 0) { Fallo "no se pudo generar la página (python tools\build_estacion.py --web)" }
}

# 2. Base local con las tablas y las credenciales
Write-Host "Preparando la base local en $Estado ..."
npx --yes $Wrangler d1 migrations apply wearme-estacion --local --persist-to $Estado 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { Fallo "no se pudieron crear las tablas (¿hay internet para descargar wrangler la primera vez?)" }
python (Join-Path $Root "tools\importar_credenciales.py") | Out-Null
if ($LASTEXITCODE -ne 0) { Fallo "no se pudieron leer las credenciales de include\wearme_credentials.h" }
$Sql = Join-Path $env:TEMP "wearme_credenciales.sql"
npx --yes $Wrangler d1 execute wearme-estacion --local --persist-to $Estado --file=$Sql 2>&1 | Out-Null
$ok = $LASTEXITCODE -eq 0
Remove-Item $Sql -ErrorAction SilentlyContinue   # trae las AppKey
if (-not $ok) { Fallo "no se pudieron cargar las credenciales en la base local" }
$n = npx --yes $Wrangler d1 execute wearme-estacion --local --persist-to $Estado --json --command "SELECT count(*) AS n FROM nodes" 2>$null | Out-String
Write-Host ("Nodos en la base local: " + ([regex]::Match($n, '"n":\s*(\d+)').Groups[1].Value))

# 3. Sesión simulada (solo funciona en localhost) y servidor
"DEV_USER=$Correo`nADMINS=$Correo`n" | Out-File -Encoding ascii (Join-Path $Web ".dev.vars")
Write-Host ""
Write-Host "Abre  http://localhost:8787  en Edge o Chrome  (usuario simulado: $Correo, administrador)"
Write-Host "Cierra antes el monitor serie de PlatformIO. Ctrl+C aqui para terminar."
Write-Host ""
try {
  npx --yes $Wrangler dev --port 8787 --persist-to $Estado
} finally {
  Remove-Item (Join-Path $Web ".dev.vars") -ErrorAction SilentlyContinue
  Remove-Item -Recurse -Force (Join-Path $Web ".wrangler") -ErrorAction SilentlyContinue
  Write-Host "Listo: se borró la sesión simulada."
}
