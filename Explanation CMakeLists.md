# Overview
Any ROS package which is to be built by `catkin` must contains `package.xml` and `CMakeLists.txt` in the root directory of the package.

## package.xml
**The `CMake` does not know about the package.xml dependencies, although `catkin` does**. If you don't specify or incorrectly specify the dependencies, you may be able to build on your system but it will not work on other system. If you specify all your package related dependencies information in this file, it can be installed automatically by catkin on other systems. This xml file contains metadata information such as author, maintainer, dependencies etc. related to the package for example

```
<package format="2">
  <name>ros_example</name>
  <version>1.0.0</version>
  <description>
    This package contains a ros example.
  </description>
  <author>Author-1</author>
  <author>Author-2</author>
  <!-- The maintainer is the one who releases tha package, not necessarilty the original author. -->
  <maintainer email="dringakn@gmail.com">Dr. -Ing. Ahmad Kamal Nasir</maintainer>
  <url type="website">http://ros.org/wiki/camera1394</url>
  <url type="repository">https://github.com/ros-drivers/camera1394.git</url>

  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
```

### Notes
- `<package format="1 or 2">` determine which format to use.
- `<depend>` This is a new tag, intended to reduce un-necessary repetition of `<build_depend>`, `<run_depend>`, `<exec_depend>`, and `<build_export_depend>` in `format-1`. The system related build binary dependencies on a linux system typically contains `lib` as a prefix and `-dev` or `-devel` postfix in tthe libraray name. For non-binary or heaer-only depencies it's simple such as `eigen`. The shared binaries doesn't contains `-dev` postfix. For system related dependencies, `<depend>` should be avoided because it forces the package's binary to depend on the development package (which is gnerally not necessary or desirable).
- `<test_depend>` it is a new tag provided in `format-2` to explicitly mention packages required for testing. In format-1 the tests related packages were also built using `build_depend` tag.
- `<doc_depend>` defines dependies needed for building package documentation e.g. `doxygen`, `epydoc`, `python-sphinx`, `rosdoc_lite`

## CMakeLists.txt
It contains mostley `CMake` and `catkin` specific commands. For your package to successfuly compile it must explicitly declares how to resolve required headers and library references.
A simpliest example file looks like this.
```
cmake_minimum_required(VERSION 2.8.3)
project(example_package) # name of the package
find_package(catkin REQUIRED COMPONENTS roscpp) # find the catkin related c++ library
find_package(Boost REQUIRED COMPONENTS thread) # find the boost thread related library
catkin_package(...)
```
A regular catkin package generally provide additional  information such as dependency, building targets, installing files and performing tests.

Any meta-package contains only following two commands:
```
find_package(catkin REQUIRED)
catkin_metapackage()
```
A meta-package is a special purpose package for grouping other packages. It must not install any code or other files. It can be used to resolve stack dependencies declared by legacy `rosbuild` packages not yet ported to `catkin`. **Catkin packages should depend directly on the packages they use, not on any meta-package** 

### Element of the CMakeLists.txt
Here are some of the commands used in a typical ROS package

1. cmake_minimum_required()
2. project()
3. find_package()
4. add_message_files(), add_service_files(), add_action_files(), all catkin-specific
5. generate_messages(), catkin-specific
6. catkin_package(), catkin-specific
7. add_library(), add_executable(), target_link_libraries()
8. install()
9. catkin_add_gtest(), catkin_add_nosetests(), add_rostest(), add_rostest_gtest(), all catkin-specific

### find_package()
it is used to find a library. It defines `CMake` variables which will be later required for compilation and linking steps. One can specify one or more depependencies within one function call e.g.
```
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs)
```
It will get all the catkin related dependencies such as roscpp std_msgs, and nav_msgs.
```
find_package(Boost REQUIRED COMPONENTS thread)
```
It will find the boost thread related library.
CMake community recommends standard names for those variables, some packages may not follow their recommendations. Sometimes, no CMake module is available, but the library’s development package provides a pkg-config file. To use that, first load the CMake PkgConfig module, then access the build flags provided by the library:
```
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED libgstreamer-0.10)
```
The first pkg_check_modules() parameter declares a prefix for CMake variables like GSTREAMER_INCLUDE_DIRS and GSTREAMER_LIBRARIES, later used for the compile and linker. The REQUIRED argument causes configuration to fail unless the following “module” is the base name of a pkg-config file provided by the library, in this example libgstreamer-0.10.pc.

**Make sure the packages are also mentioned in the `package.xml` using `<depend>` tag(s)**

### include_directories()
This command will collect all the header file paths before compilation.
For example, if your package contains a `header` folder in addition to include standard ROS headers you can specify as follows:
```
include_directories(include ${catkin_INCLUDE_DIRS})
```
 If your package also depends on other catkin packages, add ${catkin_INCLUDE_DIRS} to the list.
### catkin_package()
If you are creating a library or a package required by another ROS package, then we need to specifically export it by using the following command
```
catkin_package(CATKIN_DEPENDS angles roscpp std_msgs)
```
For system related depependencies
```
catkin_package(DEPENDS Boost GSTREAMER)
```
For this to work, you must have found those dependencies earlier, using find_package() or pkg_check_modules(), and they must define the CMake variables `${name}_INCLUDE_DIRS` and `${name}_LIBRARIES`. Note that the package name is case sensitive. While catkin packages always use a lowercase name, other packages might use uppercase (as GSTREAMER) or mixed case (like Boost).

Some packages provide variable names that do not comply with these recommendations. In that case, you must pass the absolute paths explicitly as INCLUDE_DIRS and LIBRARIES.
**Make sure the packages are also mentioned in the `package.xml` using `<depend>` tag(s)**

### add_dependencies()
Since you presumably have build targets using the message or service headers, add this to ensure all their headers get built before any targets that need them:
```
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
add_dependencies(your_library ${catkin_EXPORTED_TARGETS})
```

### add_executable()
It is used to build the binary executable for the package. Before comppiling collect all dependencies and header/library paths.
```
set(${PROJECT_NAME}_SOURCES
    src/file1.cpp
    src/file2.cpp
    src/file3.cpp
    src/file4.cpp
    src/file5.cpp
    src/file6.cpp
    src/file_etc.cpp)
add_executable(your_node ${${PROJECT_NAME}_SOURCES})
```
if the program depends on libraries provided by others (e.g. Boost, Gstreamer) then add them:
```
target_link_libraries(your_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES})
```

### install()
ROS executables are installed in a per-package directory, not the `distributions’s global bin/ directory`. List all your executables as TARGETS on an install command like this:
```
install(TARGETS your_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```
or 
```
install(TARGETS your_library your_other_library
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
```
The runtime destination is used for .dll file on Windows which must be placed in the global bin folder.

Libraries typically provide headers defining their interfaces. Please follow standard ROS practice and place all external header files under include/your_package/:
```
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
```
### add_library()
To build your library add this command to your CMakeLists.txt, listing all required C++ source files, but not the headers:
```
add_library(your_library libsrc1.cpp libsrc2.cpp libsrc_etc.cpp)
```
In catkin, libraries will be installed in a common directory shared by all the packages in that entire ROS distribution. So, make sure your library names are sufficiently unique not to clash with other packages or system libraries. It makes sense to include at least part of your package name in each library target name.

For this example, suppose your_package has a shared library build target named your_library. **On Linux, the actual file name will be something like `libyour_library.so`, perhaps with a version number suffix**.

If your library depends on additional catkin and non-catkin system libraries, include them in the target_link_libraries():
```
target_link_libraries(your_library
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES})
```
### add_message_files(DIRECTORY msg FILES MessageX.msg)
### add_service_files(DIRECTORY srv FILES ServiceX.srv)
### add_action_files(DIRECTORY action FILES ActionX.action)

For building actions, include actionlib_msgs among the dependencies:
```
find_package(catkin REQUIRED
             COMPONENTS
             actionlib_msgs
             message_generation
             std_msgs)
```
Then, generate all your message, service and action targets with this command:
```
generate_messages(DEPENDENCIES std_msgs)
```
Make sure the catkin_package() command declares your message, service and action dependencies for other packages:
```
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
```
**A good ROS practice is to collect related messages, services and actions into a separate package with no other API. That simplifies the package dependency graph.**
Every target that directly or indirectly uses one of your message headers must declare an explicit dependency:
```
add_dependencies(your_program ${${PROJECT_NAME}_EXPORTED_TARGETS})
```
If your build target also uses message or service headers imported from other catkin packages, declare those dependencies similarly:
```
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
```
**Since catkin installs message, service and action targets automatically, no extra install() commands are needed for them.**
Your package.xml must declare a <build_depend> on message_generation, and a <build_export_depend> as well as <exec_depend> on message_runtime:
```
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```
To generate actions, add actionlib_msgs as a dependency:
```
<depend>actionlib_msgs</depend>
```

### Dynamic Reconfiguration
Dynamic reconfiguration requires you to provide one or more simple Python scripts that declare the names, types, values, and ranges of the parameters you wish to configure dynamically.
You need to declare your dependency on dynamic_reconfigure:
```
<depend>dynamic_reconfigure</depend>
```
Be sure to include dynamic_reconfigure among your catkin package components:
```
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure ...)
```
Generate the reconfigure options, listing all your node’s .cfg scripts:
```
generate_dynamic_reconfigure_options(cfg/YourNode.cfg)
```
To export for other catkin packages:
```
catkin_package(CATKIN_DEPENDS dynamic_reconfigure ...)
```
Since you probably have build targets using the generated header, add this to ensure it gets built before any targets needing them:
```
add_dependencies(your_program ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(your_library ${${PROJECT_NAME}_EXPORTED_TARGETS})
```

### Python scripts and modules
Standard ROS practice is to place all executable Python programs in a package subdirectory named `nodes/` or `scripts/`. Your CMakeLists.txt should install all the scripts explictly using the special install function catkin_install_python.
```
catkin_install_python(PROGRAMS nodes/your_node scripts/another_script
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

### Installing CMake files
Sometimes your package needs to install CMake files for use by other packages. For that to work, your catkin_package() command must include a CFG_EXTRAS parameter, where you list the CMake files you want to export:
```
catkin_package(CFG_EXTRAS your_macros.cmake your_modules.cmake)
```
When another package uses find_package() on this package, the listed CMake files are automatically included.

Since these data are platform-independent, they should be installed in your package’s `share/` subtree. This example assumes your CMake sources are in the customary `cmake/` subdirectory:
```
install(FILES cmake/your_macros.cmake cmake/your_modules.cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
```
To install everything in your cmake/ subdirectory except `.svn` files:
```
install(DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE)
```

### Installing Other Files (e.g. launch)
```
install(FILES your_data your_parameters
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
```

## Configuring and running unit tests
All configuration steps related to testing should be only done conditionally when CATKIN_ENABLE_TESTING is set, which is true by default. Passing -DCATKIN_ENABLE_TESTING=0 to CMake enables configuring and building packages without any testing overhead.
When unit tests require large data files, it is better to download them from the web than include them in your source repository. This should only happen in a conditional block when testing is enabled.

Each of these files declares a target name. Running "make tests" or "make run_tests" will automatically download them all. Individual test targets do not depend on the tests target, but you can use it to download them before running one:
```
$ make tests run_tests_your_package
```
Catkin provides a convenient command, used like this:
```
if (CATKIN_ENABLE_TESTING)
  catkin_download_test_data(
    ${PROJECT_NAME}_32e.pcap
    http://download.ros.org/data/velodyne/32e.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 e41d02aac34f0967c03a5597e1d554a9)
endif()
```
The first parameter is the target name, which should normally include your project name to avoid conflicts with other packages.

The second parameter is the URL to read. If you release your package to the ROS build farm, make sure this URL is available reliably. Otherwise, tests will fail randomly, annoying everyone. Contact ros-release@lists.ros.org if you need to host some data near the build servers.

The MD5 argument is a good way to avoid testing with corrupted data.

The default destination for the file downloaded above is within your package’s build directory. The file name is the base name of the URL. You can use the DESTINATION argument to put it somewhere else.

For example, sometimes a rostest script wants to use $(find your_package) to access the test data. Here is how to put the file in devel-space, where roslaunch can resolve it.

Then, the test script can pass it as a parameter, like this:
```
<node pkg="your_package" type="your_node" name="your_node">
  <param name="data" value="$(find your_package)/tests/32e.pcap"/>
</node>
```
### Configuring gtest for C++
Gtest is the Google framework for running C++ unit tests. It is a pure C++ framework. No ROS environment is available. 
The rosunit package is needed for testing:
```
<test_depend>rosunit</test_depend>
```
Declare each gtest like this:
```
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_your_node tests/test_your_node.cpp)
  target_link_libraries(test_your_node ${catkin_LIBRARIES})
endif()
```
**This example assumes your tests are defined in the `tests/` subdirectory in your source tree.**

### onfiguring rostest
Use rostest whenever your unit tests require a roscore for running ROS nodes. Declare rostest as a test dependency, along with any other test-only dependencies:
```
<test_depend>rostest</test_depend>
```
You need a find_package() for rostest to define the necessary CMake commands. It is better not to use a second find_package(catkin ...) for test dependencies like rostest, because that would reset important catkin CMake variables, making it hard to build test programs. Place both the find_package() and your test declarations inside the conditional testing block:
```
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/your_first_rostest.test)
  add_rostest(tests/your_second_rostest.test)
endif()
```
In case you have a test that accepts arguments, you can pass them like this:
```
add_rostest(tests/your_rostest.test ARGS arg1:=true arg2:=false)
```
If your rostest needs extra data in order to run, you can use the catkin_download_test_data() to download the data. Read more about Downloading test data. Then you can add a dependency between the rostest target and the target from catkin_download_test_data(), in order to download the data before the rostest runs:
```
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_download_test_data(
    ${PROJECT_NAME}_32e.pcap
    http://download.ros.org/data/velodyne/32e.pcap
    MD5 e41d02aac34f0967c03a5597e1d554a9)

  add_rostest(tests/your_rostest.test DEPENDENCIES ${PROJECT_NAME}_32e.pcap)
endif()
```

### Running unit tests
All unit tests can be run by invoking the run_tests target: 
```
catkin_make run_tests
```
It will make sure that the tests target which builds all unit tests is invoked before.
