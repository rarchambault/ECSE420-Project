# ECSE 420 Project

## Setting up the solution
Upon downloading the project for the first time, you will have to run a setup script which will generate required solution files.
Follow the corresponding instructions for your environment.

### Microsoft Visual Studio
* Run `build-VisualStudio2022.bat`
* double click the `.sln` file that is generated
* develop your game
* you are good to go

### Linux Users
* CD into the build folder
* run `./premake5 gmake2`
* CD back to the root
* run `make`
* you are good to go

### MacOS Users
* CD into the build folder
* run `./premake5.osx gmake2`
* CD back to the root
* run `make`
* you are good to go

## Setting up vcpkg
Some packages in this project are added during build by **vcpkg**. You may have to download vcpkg in order to get those packages.
If you run into any issues related to packages not being found, follow the instructions below.

**Note**: The manifest file and its dependencies are already pushed in the repository, so you will not need ot create them again.
https://learn.microsoft.com/en-us/vcpkg/get_started/get-started-msbuild?pivots=shell-powershell