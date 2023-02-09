
# C/C++ library for MEKF 

This is a C++ custom library with a C wrapper to make it easier to interface with our main.c .

The underlying linear algebra library should ideally have support for : identity matrix, zero matrix, constant * matrix, cross operator, transpose, exponential matrix. 
In the event of a change in the linear algebra library, the most important feature is the exponential matrix. 