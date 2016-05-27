ACSIMSEP
========
an AC SIMulator) of SEParation: Simulating cascading failure in power systems

Introduction
------------
ACSIMSEP is a quasi-steady-state simulator of cascading failures in large-scale power transmission systems. The code is written primarly by Paul Hines at the University of Vermont (uvm.edu/~phines).

Installation
------------
To install ACSIMSEP follow the following proceedure:
* First make a local copy of the repository using something like ```git@github.com:phines/acsimsep.git```
* Navigate to the ```ThirdPaty/SuiteSparse``` directory.
* Install the key Third Party codes:
* Install SuiteSparse by navigating to the ```acsimsep/ThirdParty/SuiteSparse``` directory and then: ```make``` and then ```sudo make install```. This installs the library and include files under ```/usr/local/```.
* Install Coin-Bonmin and its friends: ```cd ../Coin-Bonmin/``` and then ```./configure``` and then ```make``` and then ```make install```
* Once the third party code is installed, you should be able to compile ACSIMSEP by navigating to the root acsimsep directory and then issuing ```./configure``` and then ```make``` and then ```make install```.
* This will install several test programs in the ```acsimsep/bin``` directory. Hopefully they work.

Other documentation
------------
Some day I'll write some :)
