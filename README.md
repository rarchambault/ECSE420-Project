# ECSE 420 Project

## Prerequisites
_The project only has been fully tested on Windows with an Intel processor and a NVDIA GPU._
- [Visual Studio 2022](https://visualstudio.microsoft.com/) with C/C++ configured
- [CUDA Toolkit](https://docs.nvidia.com/cuda/cuda-installation-guide-microsoft-windows/)
- [Intel CPU Runtime for OpenCL Applications](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-cpu-runtime-for-opencl-applications-with-sycl-support.html)

## Installation

### Set up the solution
Upon downloading the project for the first time, run a setup script which will generate required solution files for Visual Studio.

#### Setting up vcpkg
1. Clone vcpkg
    ```bat
    git clone https://github.com/microsoft/vcpkg.git
    ```
2. Run the bootstrap script
    ```bat
    cd vcpkg
    .\bootstrap-vcpkg.bat
    ```
3. Now set the `VCPKG_ROOT` environment variable and add it to path.
    ```bat
    $env:VCPKG_ROOT = "C:\path\to\vcpkg"
    $env:PATH = "$env:VCPKG_ROOT;$env:PATH"
    ```
4. Verify your installation
    ```bat
    vcpkg --version
    ```
5. Integrate with Visual Studio MSBuild
    ```bat
    vcpkg integrate install
    ```
    This should output:
    ```
    All MSBuild C++ projects can now #include any installed libraries. Linking will be handled automatically. Installing new libraries will make them instantly available.
    ```

#### Set up the Visual Studio solution
1. Run `build-VisualStudio2022.bat`.
2. Open the _.sln_ solution file that was generated in Visual Studio.
3. In **Project > Properties**, navigate to **Configuration Properties > vcpkg**, and set `Use vcpkg Manifest` to `Yes`.
4. In **Solution Explorer**, right click on **Source Files > Add > Existing item** and add `src/particle.cl` and `src/particle.cu`.
5. In **Solution Explorer**, right click on **ECSE420-Project > Build Dependencies > Build Customizations** and click the box for your CUDA installation.
6. In **Solution Explorer**, right click on **particle.cu > Properties** 
4. The project can now be built and ran like any other Visual Studio project. In `Item Type`, select `CUDA C/C ++`.

**Note**: The manifest file and its dependencies are already pushed in the repository, so you will not need to create them again. More info on the setup [here](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started-msbuild?pivots=shell-powershell).

## Run the simulation

1. In `main.c`, select your preferred execution mode and set the `executionMode` variable:
    - EXECUTION_SEQUENTIAL
    - EXECUTION_CPU_THREADING
    - EXECUTION_GPU_OPENCL
    - EXECUTION_CPU_GRAPHICS_OPENCL
    - EXECUTION_CPU_OPENCL
    - EXECUTION_GPU_CUDA
2. Build and run the code