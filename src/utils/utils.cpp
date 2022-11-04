#include "utils/utils.h"

void removeRowAndColumn(Eigen::MatrixXd& M, int index) {
    unsigned int dimX = M.cols();
    // cout << "Removing index: " << index<< endl;
    M.block(index,0,dimX-index-1,dimX) = M.bottomRows(dimX-index-1).eval();
    M.block(0,index,dimX,dimX-index-1) = M.rightCols(dimX-index-1).eval();
    M.conservativeResize(dimX-1,dimX-1);
}
