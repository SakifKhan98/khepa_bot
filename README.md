## Khepa Bot

Khepa Bot is a ROS-based robot package designed for easy setup and customization. This repository provides the essential structure and files to get your robot project up and running quickly.

### Features

- ROS package template for rapid development
- Organized directory structure for source code, configuration, and launch files
- Example files to ensure compatibility with CMake and Git

### Getting Started

1. **Clone the repository:**

```bash
git clone https://github.com/yourusername/khepa_bot_ws.git
```

2. **Install dependencies:**

```bash
cd khepa_bot_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace:**

```bash
ros2 launch khepa_bot launch_sim.launch.py
```

### Customization

- Rename instances of `khepa_bot` to match your project name using your IDE's "Find and Replace" feature.
- Remove or modify example files as needed, and update `CMakeLists.txt` if you change the directory structure.

### Directory Structure

```
khepa_bot_ws/
└── src/
   └── khepa_bot/
      ├── src/           # Source code
      ├── launch/        # Launch files
      ├── config/        # Configuration files
      └── CMakeLists.txt # Build configuration
```

### Contributing

Contributions are welcome! Please open issues or submit pull requests for improvements.

### License

This project is licensed under the MIT License.
