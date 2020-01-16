#include <iostream>   // std::cout
#include <iomanip>    // std::fixed, std::setprecision to print doubles/floats

#include "cpr_graphmatching_path.h"
#include "cpr_matrices.h"
#include "cpr_main.h"

void fill_adjacency_matrices(Eigen::MatrixXi &src_adj, Eigen::MatrixXi &dst_adj)
{
  // house with bottomleft-topright diagonal
  src_adj << 0, 1, 0, 0, 1,     //   0
             1, 0, 1, 1, 0,     // 4  /1
             0, 1, 0, 1, 0,     // 3/  2
             0, 1, 1, 0, 1,
             1, 1, 0, 1, 0;

  // house with topleft-bottomright diagonal
  dst_adj << 0, 1, 0, 0, 1,     //   0
             1, 0, 1, 0, 0,     // 4\  1
             0, 1, 0, 1, 1,     // 3  \2
             0, 0, 1, 0, 1,
             1, 1, 1, 1, 0;
}

// fill src_esf with pseudo-random numbers in [0, 1]
// fill dst_esf with a small perturbation of src_esf
void fill_esf_descr(ESFDescriptors &src_esf, ESFDescriptors &dst_esf)
{
  for (KeyT i = 0; i < 5; ++i)
  {
    for (std::size_t j = 0; j < 640; ++j)
    {
      // src = pseudo-random in [0, 1]
      src_esf[i].push_back(static_cast<float>((i % 3 + 1) * (j % 17 + 1)) / 51.0f);

      // dst = src + pseudo-random in [-0.0185, 0.02]
      dst_esf[i].push_back(src_esf[i][j] + ((static_cast<float>((i % 4 + 1) * (j % 7 + 1)) - 14.0f) / 700.0f));
    }
  }
}

void fill_edge_descr(EdgeDescriptors &src_ed, EdgeDescriptors &dst_ed)
{
  src_ed[std::make_pair(0,1)] = std::make_tuple(3.14/4.0, 3.14/4.0, 0, 1.4);
  src_ed[std::make_pair(1,2)] = std::make_tuple(0.0, 3.14/2.0, 0, 1.0);
  src_ed[std::make_pair(2,3)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  src_ed[std::make_pair(3,4)] = std::make_tuple(0, 3.14/2.0, 0, 1.0);
  src_ed[std::make_pair(4,0)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);
  src_ed[std::make_pair(1,4)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  src_ed[std::make_pair(1,3)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);
  src_ed[std::make_pair(1,0)] = src_ed[std::make_pair(0,1)];
  src_ed[std::make_pair(2,1)] = src_ed[std::make_pair(1,2)];
  src_ed[std::make_pair(3,2)] = src_ed[std::make_pair(2,3)];
  src_ed[std::make_pair(4,3)] = src_ed[std::make_pair(3,4)];
  src_ed[std::make_pair(0,4)] = src_ed[std::make_pair(4,0)];
  src_ed[std::make_pair(4,1)] = src_ed[std::make_pair(1,4)];
  src_ed[std::make_pair(3,1)] = src_ed[std::make_pair(1,3)];
  dst_ed[std::make_pair(0,1)] = std::make_tuple(3.14/4.0, 3.14/4.0, 0, 1.4);
  dst_ed[std::make_pair(1,2)] = std::make_tuple(0.0, 3.14/2.0, 0, 1.0);
  dst_ed[std::make_pair(2,3)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  dst_ed[std::make_pair(3,4)] = std::make_tuple(0, 3.14/2.0, 0, 1.0);
  dst_ed[std::make_pair(4,0)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);
  dst_ed[std::make_pair(1,4)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  dst_ed[std::make_pair(4,2)] = std::make_tuple(3.14/4.0, 3.14/4.0, 0, 1.4);
  dst_ed[std::make_pair(1,0)] = dst_ed[std::make_pair(0,1)];
  dst_ed[std::make_pair(2,1)] = dst_ed[std::make_pair(1,2)];
  dst_ed[std::make_pair(3,2)] = dst_ed[std::make_pair(2,3)];
  dst_ed[std::make_pair(4,3)] = dst_ed[std::make_pair(3,4)];
  dst_ed[std::make_pair(0,4)] = dst_ed[std::make_pair(4,0)];
  dst_ed[std::make_pair(4,1)] = dst_ed[std::make_pair(1,4)];
  dst_ed[std::make_pair(2,4)] = dst_ed[std::make_pair(4,2)];
}

int main(void)
{
  ESFDescriptors  src_esf;
  ESFDescriptors  dst_esf;
  fill_esf_descr(src_esf, dst_esf);

  EdgeDescriptors src_ed;
  EdgeDescriptors dst_ed;
  fill_edge_descr(src_ed, dst_ed);

  Eigen::MatrixXi src_adj(5,5);
  Eigen::MatrixXi dst_adj(5,5);
  fill_adjacency_matrices(src_adj, dst_adj);

  VertexSimilarityMatrix vsim_mat(src_esf, dst_esf);
  EdgeSimilarityMatrix esim_mat(src_ed, dst_ed);

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Vertex similarity matrix:" << std::endl;
  std::cout << vsim_mat.m << std::endl;
  std::cout << "Edge similarity matrix:" << std::endl;
  std::cout << esim_mat.m << std::endl;

  GraphMatchingPath gm(&vsim_mat.m, &esim_mat, &src_adj, &dst_adj);

  Eigen::MatrixXd x(5,5);
  x << 0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2;

  gm.frankWolfe(0.0, &x, &x);

  std::cout << x << std::endl;

  return (0);
}
