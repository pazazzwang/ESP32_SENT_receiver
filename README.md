| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# _ESP32_SENT_receiver_

This is a ESP32 project dedicated to read SENT protocal signal. Since ESP32 is cheap and has very fast sampling rate. It can read SENT signal。
Author is not a professional coder and don't know how to use github, but I uploaded this project anyway.
The main.c in main folder is the essential code, it allow us to read severial channel SENT signal at once.
But the SENT frame won't be continuous if it read 2 or more signals. If you want to parse the serial channel of the SENT signal, you need to only read one channel, so ESP32 could have the processing speed to read countinuous SENT frames.

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
