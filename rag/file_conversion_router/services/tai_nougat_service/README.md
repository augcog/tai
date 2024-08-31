# TAI Nougat Service

The TAI Nougat Service is an optimized and refined version of the original [Meta Nougat](https://github.com/facebookresearch/nougat) service, developed to enhance performance and maintainability.

## Key Enhancements

- **Performance Optimization**
  - Implemented **Dependency Injection** to reduce repeated model loading, resulting in significant time savings.
    - Testing on developer laptops within our team shows a reduction of 4 to 8 seconds per PDF, depending on the specific device.
  - **Better Support for Apple Chips**. It supports hard-ware detection and can use MLX instead of Torch for Apple Chips. This saves conversion time significantly for Apple Chip Devices.
    - By @perryzjc 's testing, a M1 Pro Chip Macbook Pro takes less than 30 seconds to convert 10 pages PDF, compared to original 10 mins in @perryzjc's device.

- **Code Refactoring**
  - Improved the readability and maintainability of the codebase to facilitate easier future development and collaboration.
