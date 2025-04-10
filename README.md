# SLAM Systems Development Setup

This repository contains multiple SLAM (Simultaneous Localization and Mapping) systems including ORB-SLAM3 and XFeatSLAM. This README provides step-by-step instructions on how to clone, build, and set up the repository for development.

## Repository Structure

- `ORBSLAM3/`: ORB-SLAM3 implementation (C++14)
- `XFeatSLAM/`: XFeatSLAM implementation (C++17)
- `Thirdparty/`: Contains all dependencies
- `Examples/`: Example programs for both SLAM systems
- `Vocabulary/`: Feature vocabulary for SLAM systems
- `build/`: Build directory (created during build process)
- `lib/`: Output directory for compiled libraries

## System Requirements

- Ubuntu 18.04 or higher (recommended)
- C++17 compatible compiler (GCC 7+ or Clang 5+)
- CMake 3.10 or higher
- Git LFS (for tracking large files)

## Dependencies

This project depends on:
- OpenCV 4.1.0
- Eigen 3.4.0
- Pangolin 0.6
- DBoW2
- g2o (Customized version)
- Sophus
- PyTorch (for XFeatSLAM)

## Getting Started

### 1. Clone the Repository

```bash
# Clone the main repository
git clone https://your-repository-url.git
cd slam-systems

# If using Git LFS (for large binary files)
git lfs install
git lfs pull
```

### 2. Build Dependencies

All dependencies are managed through the `Thirdparty/build_deps.sh` script:

```bash
cd Thirdparty
chmod +x build_deps.sh
./build_deps.sh
```

This script will build all required dependencies:
- DBoW2
- Eigen 3.4.0
- g2o
- Pangolin 0.6
- OpenCV 4.1.0
- Sophus

For XFeatSLAM, you'll also need PyTorch. The setup differs based on your system configuration, so please follow the instructions at [PyTorch's official site](https://pytorch.org/get-started/locally/).

### 3. Build the SLAM Systems

Once all dependencies are built, you can build the SLAM systems:

```bash
cd ..  # Back to repository root
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

You can specify which SLAM system to build using the following options:
- `-DBUILD_ORBSLAM3=ON|OFF`: Build ORB-SLAM3 (default: ON)
- `-DBUILD_XFEATSLAM=ON|OFF`: Build XFeatSLAM (default: ON)

For example, to build only ORB-SLAM3:
```bash
cmake -DBUILD_XFEATSLAM=OFF ..
```

### 4. Running Examples

Example executables are built in the `Examples/` directory:

#### ORB-SLAM3 Examples
```bash
# RGB-D TUM dataset
./Examples/ORB/rgbdORB_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association

# Monocular TUM dataset
./Examples/ORB/monoORB_tum path_to_vocabulary path_to_settings path_to_sequence
```

#### XFeatSLAM Examples
```bash
# RGB-D TUM dataset
./Examples/XF/rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association

# Monocular TUM dataset
./Examples/XF/mono_tum path_to_vocabulary path_to_settings path_to_sequence
```

## Development Notes

### Excluded Files/Directories

The following directories are excluded from git tracking:
- `build/`: Build artifacts
- `lib/`: Compiled libraries
- `Thirdparty/pytorch/`: PyTorch libraries (must be installed separately)
- `Thirdparty/libs/`: Compiled third-party libraries
- `**/data/`: Data directories
- `**/dataset*/`: Dataset directories
- Object files, shared libraries, and other compiled artifacts

### Common Development Tasks

1. **Adding a new feature to ORB-SLAM3**:
   - Modify files in `ORBSLAM3/src/` and `ORBSLAM3/include/`
   - Rebuild with `cmake .. && make -j$(nproc)` from the `build` directory

2. **Adding a new feature to XFeatSLAM**:
   - Modify files in `XFeatSLAM/src/` and `XFeatSLAM/include/`
   - Rebuild with `cmake .. && make -j$(nproc)` from the `build` directory

3. **Creating a new example**:
   - Create your example in `Examples/` directory
   - Add it to the appropriate CMakeLists.txt
   - Rebuild with `cmake .. && make -j$(nproc)` from the `build` directory

## Troubleshooting

1. **Missing dependencies**:
   - Ensure you've run the `build_deps.sh` script
   - Check if all dependencies in `Thirdparty/` have been built successfully

2. **Build errors**:
   - Make sure you have the right compiler version
   - Ensure all dependencies are correctly built

3. **Runtime errors**:
   - Check that all path arguments to examples are correct
   - Verify that datasets are in the expected format

## Contributing

1. Create a feature branch
2. Make your changes
3. Run tests to ensure everything works
4. Submit a pull request

## License

Please refer to the LICENSE file for licensing information. 