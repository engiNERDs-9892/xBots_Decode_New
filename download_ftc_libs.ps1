# PowerShell script to download FTC SDK libraries manually
$baseUrl = "https://repo1.maven.org/maven2/org/firstinspires/ftc"
$version = "11.0.0"
$libsDir = "libs"

# Create libs directory if it doesn't exist
if (!(Test-Path $libsDir)) {
    New-Item -ItemType Directory -Path $libsDir -Force
}

# List of FTC libraries to download
$libraries = @(
    "Inspection",
    "Blocks", 
    "RobotCore",
    "RobotServer",
    "OnBotJava",
    "Hardware",
    "FtcCommon",
    "Vision"
)

Write-Host "Downloading FTC SDK libraries version $version..."

foreach ($lib in $libraries) {
    $jarName = "$lib-$version.jar"
    $url = "$baseUrl/$lib/$version/$jarName"
    $outputPath = "$libsDir\$jarName"
    
    Write-Host "Downloading $lib..."
    try {
        Invoke-WebRequest -Uri $url -OutFile $outputPath -UseBasicParsing
        Write-Host "Downloaded $jarName" -ForegroundColor Green
    }
    catch {
        Write-Host "Failed to download $jarName" -ForegroundColor Red
        Write-Host $_.Exception.Message
    }
}

Write-Host "Download complete!"