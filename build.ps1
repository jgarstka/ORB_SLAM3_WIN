# build ORB-SLAM2 on Windows

if ( $args.count -ne 4 )
{
    write-host "Syntax:"
    write-host "build.ps1 <max_build_threads> <build directory> <path to vcpkgs' CMake toolchain file> <OpenCV directory>"
    Return
}

$src_path    = Get-Location
$max_threads = $args[0]
$dst_path    = $args[1]
$vcpkg_file  = $args[2]
$opencv_dir  = $args[3]

write-host "Config:"
write-host " - source:         $src_path"
write-host " - destination:    $dst_path"
write-host " - toolchain file: $vcpkg_file"
write-host " - OpenCV dir:     $opencv_dir"


# Test if CMakeLists.txt exists
if ( -Not (Test-Path -Path $src_path"\CMakeLists.txt") )
{
    write-host "Unable to find CMakeLists.txt in $src_path"
    return
}

# Update source path with absolute path
$src_path = Get-Location

# check if( destination folder already exists or create is
if ( -Not (Test-Path -Path $dst_path) )
{
    "Destination folder does not exist, creating a new folder: $dst_path"
    [void](New-Item -Path $dst_path -ItemType Directory)

    if ( -Not (Test-Path -Path $dst_path) )
    {
        write-host "Unable to create destination folder $dst_path"
        return
    }
}

# Update destination path with absolute path
cd "$dst_path"
$dst_path = Get-Location

# Switch to latest VS environment
$vsPath = &"${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationpath

Import-Module (Get-ChildItem $vsPath -Recurse -File -Filter Microsoft.VisualStudio.DevShell.dll).FullName
Enter-VsDevShell -VsInstallPath $vsPath -SkipAutomaticLocation -DevCmdArguments "-arch=x64 -host_arch=x64"

# Check if cmake is available
if ( -Not ( Get-Command 'cmake' ) )
{
    write-host "Unable to find cmake"
    return
}

# Change build environment to en-US to avoid internal error in CL.exe
$env:VSLANG = "1033"
chcp 1252

# Run cmake
write-host "Configuring cmake project"
cmake "$src_path" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$vcpkg_file" -DOpenCV_DIR="$opencv_dir" -DCMAKE_BUILD_TYPE="Release"

# Start build
write-host "Starting Ninja build"
ninja -j $max_threads

# Return to source directory
cd "$src_path"
