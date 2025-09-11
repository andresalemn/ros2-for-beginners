# The General ROS2 Command-Line Workflow

This guide explains the fundamental rules and rhythm of working with ROS2 from the command line. It is not about *what* to type for a specific task, but *why* and *when* you perform certain actions.

## The Core Philosophy: Separation of Code, Build, and Install

Your ROS2 workspace (`ros2_ws`) has three critical directories that you interact with:

1.  **`src/` (Source):** This is where you live. All your code, package configurations, and launch files go here. This is the only directory you should be manually editing.

2.  **`build/` (Build):** This is `colcon`'s workshop. When you run `colcon build`, it takes your source code, compiles it, and prepares it for installation. You should never edit anything in here directly.

3.  **`install/` (Install):** This is the finished product. After a successful build, `colcon` places all the final executables, libraries, and other files here. Your ROS2 environment actually runs everything from the `install` directory, **not** from your `src` directory.

**Key Idea:** The code you write in `src` is not the code you run. You run the *installed* version of your code, which lives in `install`.

---

## The State of Your Terminal: Why Sourcing is Everything

**Every new terminal you open is completely ignorant of your ROS2 workspace.**

When you open a new terminal, it has no idea where your executables are or what ROS2 packages exist. 

**Sourcing is the act of educating your terminal.**

When you run `source install/setup.bash`, you are executing a script that tells your *current terminal session* all about your workspace. It sets up environment variables so that the `ros2` command knows where to find your packages, nodes, and messages.

This leads to the most important rules:

*   **Rule 1: You MUST source the setup file in EVERY new terminal you open** if you want to use your workspace's packages.
*   **Rule 2:** Sourcing only affects the current terminal. It does not affect other terminals that are already open or that you open in the future.

---

## The Immutable Law of ROS2 Development

For any and every change you make to your source code, you must follow this three-step process from the root of your workspace (`~/ros2_ws`):

1.  **`colcon build`**
    *   **What it does:** Compiles your code from `src` and places the new, updated executables and files into the `install` directory.
    *   **When to run it:** After ANY change in the `src` directory. This includes editing a Python file, a C++ file, `package.xml`, `CMakeLists.txt`, or a launch file.

2.  **`source install/setup.bash`**
    *   **What it does:** Updates your current terminal's environment to make it aware of the changes you just made in the `install` directory.
    *   **When to run it:** After every `colcon build`.

3.  **`ros2 run ...` / `ros2 launch ...`**
    *   **What it does:** Runs the code.
    *   **When to run it:** Only after you have built your changes and sourced the environment.

Think of it as a single, atomic action: **Edit -> Build -> Source -> Run.**

### Common Scenarios

*   **Problem:** "I just changed my Python node, but it's running the old version of the code!"
    *   **Solution:** You forgot to run `colcon build`. (Or you can use the `--symlink-install` tip below!)

*   **Problem:** "I just built my code, but `ros2 run` says it can't find my new node!"
    *   **Solution:** You forgot to `source install/setup.bash` after building.

*   **Problem:** "I opened a new terminal and `ros2 run` can't find my package!"
    *   **Solution:** You forgot to `source install/setup.bash` in the new terminal.

---

## Efficiency Tips

*   **Build Selectively:** If your workspace is large, rebuilding everything can be slow. You can build only the package you changed:
    ```bash
    colcon build --packages-select my_package_name
    ```
    **Important:** Even when you do this, you still source the main `install/setup.bash` file, not anything specific to the package.

*   **Create a Terminal Alias:** Typing `source install/setup.bash` is tedious. You can create a shortcut. Add this to your `~/.bashrc` file:
    ```bash
    alias src_ros="source ~/ros2_ws/install/setup.bash"
    ```
    Now, you can just type `src_ros` in any new terminal.

---

## Advanced Tip: Speeding Up Python Development with `--symlink-install`

The "Immutable Law" has an exception that can make **Python** development much faster.

When you run the build command, you can give it the `--symlink-install` option:

```bash
colcon build --symlink-install
```

**What it does:** Instead of *copying* your Python scripts and launch files from `src/` to `install/`, this command creates a symbolic link (a "shortcut"). The file in the `install` directory now points directly to the original file in the `src` directory.

**The Result (for Python code and config files):**
You can edit your Python node or launch file in the `src` directory, and the changes are **instantly** reflected. You can skip the `colcon build` step for simple code edits.

The workflow becomes: **Edit -> Run**.

**IMPORTANT LIMITATIONS (When you STILL need to `colcon build`):**

*   **This does NOT work for C++:** C++ is a compiled language. If you change a `.cpp` or `.hpp` file, you **MUST** run `colcon build`.
*   **This does NOT work for configuration changes:** If you add a new node in `setup.py`, change `package.xml`, or add new message files, you **MUST** run `colcon build` so it can properly update the install directory structure.

It is a great habit to use `colcon build --symlink-install` for Python-heavy workspaces.