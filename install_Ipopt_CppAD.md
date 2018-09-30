## Installing Ipopt and CppAD

### Installation Process

1. [Download](https://www.coin-or.org/download/source/Ipopt/) the appropriate version of Ipopt (3.12.7 or higher) from the link below.  You may also use wget or a similiar command to download the source from the command line (see Linux instructions).
2. Follow the instructions for your environment

* [Ipopt](https://projects.coin-or.org/Ipopt)
  * **Mac:**
    ```
      brew tap brewsci/science
      brew install ipopt --with-openblas
    ```
    Alternatively, you may be able to install directly from this repository with the below, although you may be missing certain dependencies that need to be carried over from the above:
    ```
    brew tap udacity/CarND-MPC-Project https://github.com/udacity/CarND-MPC-Project
    brew install ipopt --with-openblas
    ```

 - **For Linux and Windows Ubuntu BASH** Please note that for any particular command, including execution of ```.sh``` scripts, it may be necessary to add ```sudo``` prior to the command.  It is also a good practice to run ```sudo apt-get update``` prior to installation of new libraries.

  * **Linux:**
    * ```sudo apt-get install gfortran```
    *  ```apt-get install unzip```
    * ```wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip```
    * Call `install_ipopt.sh` with the source directory as the first argument, ex: ```./install_ipopt.sh Ipopt-3.12.7``` or ```bash install_ipopt.sh Ipopt-3.12.7```

* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
