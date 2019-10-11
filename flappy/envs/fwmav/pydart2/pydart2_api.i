%module pydart2_api

%{
  #define SWIG_FILE_WITH_INIT
  #include "pydart2_api.h"
  #include "pydart2_world_api.h"
  #include "pydart2_skeleton_api.h"
  #include "pydart2_bodynode_api.h"
  #include "pydart2_dof_api.h"
  #include "pydart2_joint_api.h"
  #include "pydart2_shape_api.h"
%}

/* Include the NumPy typemaps library */
%include "numpy.i"

%init %{
  import_array();
%}

%include "std_vector.i"

/* SWIG template for get_rand_list(int length) C++ routine */
namespace std {
  %template(IntVector) vector<int>;
}

%apply (double IN_ARRAY1[ANY]) {(double inv3[3])};
%apply (double IN_ARRAY1[ANY]) {(double inv3_2[3])};
%apply (double IN_ARRAY1[ANY]) {(double inv4[4])};
%apply (double IN_ARRAY1[ANY]) {(double inv6[6])};

%apply (double ARGOUT_ARRAY1[ANY]) {(double outv3[3])};
%apply (double ARGOUT_ARRAY1[ANY]) {(double outv4[4])};
%apply (double ARGOUT_ARRAY1[ANY]) {(double outv6[6])};

%apply (double IN_ARRAY2[ANY][ANY]) {(double inv33[3][3])};
%apply (double IN_ARRAY2[ANY][ANY]) {(double inv44[4][4])};
%apply (double ARGOUT_ARRAY2[ANY][ANY]) {(double outv33[3][3])};
%apply (double ARGOUT_ARRAY2[ANY][ANY]) {(double outv44[4][4])};

%apply (double* IN_ARRAY1, int DIM1) {(double* inv, int ndofs)};
%apply (double* IN_ARRAY1, int DIM1) {(double* inv1, int indofs1)};
%apply (double* IN_ARRAY1, int DIM1) {(double* inv2, int indofs2)};

%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv, int ndofs)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv1, int ondofs1)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv2, int ondofs2)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv, int nout)};
%apply (double* INPLACE_ARRAY2, int DIM1, int DIM2) {(double* outm, int nrows, int ncols)};

%include "pydart2_api.h"
%include "pydart2_world_api.h"
%include "pydart2_skeleton_api.h"
%include "pydart2_bodynode_api.h"
%include "pydart2_dof_api.h"
%include "pydart2_joint_api.h"
%include "pydart2_shape_api.h"
