param(
    [string]$Targets = "",          # space-separated cmake targets; empty = all
    [string]$Filter = "",           # ctest -R regex; empty = all
    [string]$Config = "x64-Debug"
)
# Helper: configure + build + ctest inside a VS developer environment.
$vcvars = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
$bd = "out/build/$Config"
$build = "cmake --build $bd"
if ($Targets) { $build += " --target $Targets" }
$test = "ctest --test-dir $bd --output-on-failure"
if ($Filter) { $test += " -R `"$Filter`"" }
$cmd = "`"$vcvars`" >nul && cmake -S . -B $bd -G Ninja && $build && $test"
cmd /c $cmd
