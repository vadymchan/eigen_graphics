# eigen_graphics

A graphics extension library for the Eigen math library, providing additional functionality for computer graphics.

## Features

- Translation, scaling, and rotation (Euler angles and Rodrigues' rotation formula)
- View matrix calculation
- Orthographic and perspective projection matrices with configurable parameters
- Frustum computation
- Support for various configurations:
  - Row-major and column-major matrix storage
  - Right-handed and left-handed coordinate systems
  - Depth values ranging from -1 to 1 (No) or 0 to 1 (Zo)
- Functions for transforming points and vectors

## Requirements

- C++17 or later
- Eigen math library (version 3.4.0 or later)

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/vadym_chan/eigen_graphics.git
   ```
2. Include the necessary headers in your project:
   ```cpp
   #include <eigen_graphics/all.h>
   ```

## Usage

Here's a simple example of how to use eigen_graphics:

```cpp
#include <eigen_graphics/all.h>

int main() {
    // Create a perspective projection matrix
    Eigen::Matrix4f projection = Eigen::Graphics::perspective(45.0f, 16.0f / 9.0f, 0.1f, 100.0f);

    // Create a view matrix
    Eigen::Vector3f eye(0, 0, 5);
    Eigen::Vector3f target(0, 0, 0);
    Eigen::Vector3f up(0, 1, 0);
    Eigen::Matrix4f view = Eigen::Graphics::lookAt(eye, target, up);

    // Combine the matrices
    Eigen::Matrix4f mvp = projection * view;

    // Transform a point
    Eigen::Vector4f point(1, 2, 3, 1);
    Eigen::Vector4f transformedPoint = mvp * point;

    return 0;
}
```

More examples can be found in the `examples/` directory.

## TODO

- [ ] Add support for geometry primitives:
  - [ ] Triangle
  - [ ] Cube
  - [ ] Sphere
  - [ ] Cylinder
  - [ ] Plane
- [ ] Implement utility functions:
  - [ ] degreesToRadians
  - [ ] radiansToDegrees
- [ ] Improve documentation and comments
- [ ] Add more examples demonstrating various use cases
- [ ] Enhance error handling and input validation
- [ ] Add benchmarks for testing functionality

## Contributing

Contributions are welcome! If you find a bug or have a feature request, please open an issue on the GitHub repository. If you'd like to contribute code, please follow these steps:

1. Fork the repository
2. Create a new branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -m 'Add your feature'`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Open a pull request

## License

eigen_graphics is licensed under the [MIT License](LICENSE).

## Contact

If you have any questions or suggestions, feel free to contact me: vadymchan@gmail.com.
