#include <iostream>   // std::cout
#include <iomanip>    // std::fixed, std::setprecision to print doubles/floats

#include "cpr_graphmatching_path.h"
#include "cpr_matrices.h"
#include "cpr_main.h"

void testmultiple_fill_adjacency_matrices(MatrixInt &src_adj, MatrixInt &dst_adj)
{
  // house with 5 nodes and 7 edges
  //   0
  // 4  /1
  // 3/  2
  src_adj << 0, 1, 0, 0, 1,  // roof
             1, 0, 1, 1, 1,  // topright
             0, 1, 0, 1, 0,  // bottomright
             0, 1, 1, 0, 1,  // bottomleft
             1, 1, 0, 1, 0;  // topleft

  // house+tail with 6 nodes and 8 edges
  //   0
  // 5  /1
  // 4/  2 _ 3
  dst_adj << 0, 1, 0, 0, 0, 1,  // roof
             1, 0, 1, 0, 1, 1,  // topright
             0, 1, 0, 1, 1, 0,  // bottomright
             0, 0, 1, 0, 0, 0,  // tail
             0, 1, 1, 0, 0, 1,  // bottomleft
             1, 1, 0, 0, 1, 0;  // topleft
}

// fill src_esf[0,1,2,3,4] with pseudo-random numbers in [0, 1]
// fill dst_esf[0,1,2,4,5] with a small perturbation of src_esf
// dst_esf[3] = pseudo-random
void testmultiple_fill_esf_descr(ESFDescriptors &src_esf, ESFDescriptors &dst_esf)
{
  int random = 197546;

  // src[0,1,2,3,4]
  for (KeyT i_src = 0; i_src < 5; ++i_src)
  {
    for (std::size_t j = 0; j < 640; ++j)
    {
      random = (random * 7 + 19 + i_src + j) % 31;
      // src = pseudo-random in [0, 1]
      src_esf[i_src].push_back(static_cast<float>(random) / 30.0f);
    }
  }

  // dst[0,1,2,4,5]
  for (KeyT i_dst = 0; i_dst < 6; ++i_dst)
  {
    KeyT i_src = (i_dst <= 2 ? i_dst : i_dst - 1);  // src 0 1 2 3 4 -> dst 0 1 2 4 5
    for (std::size_t j = 0; j < 640; ++j)
    {
      random = (random * 7 + 19 + i_dst + j) % 31;
      // dst = src + pseudo-random in [-0.02, 0.02]
      dst_esf[i_dst].push_back(src_esf[i_src][j] + (static_cast<float>(random) / (30.0f * 25.0f) - 0.02f));
    }
  }

  // dst[3]
  for (std::size_t j = 0; j < 640; ++j)
  {
    random = (random * 7 + 19 + 3 + j) % 31;
  // dst[3] = pseudo_random in [0,1]
    dst_esf[3][j] = (static_cast<float>(random) / 30.0f);
  }
}

void testmultiple_fill_edge_descr(EdgeDescriptors &src_ed, EdgeDescriptors &dst_ed)
{
  src_ed[std::make_pair(0,1)] = std::make_tuple(3.14/4.0, 3.14/4.0, 0, 1.4);
  src_ed[std::make_pair(1,2)] = std::make_tuple(0.0, 3.14/2.0, 0, 1.0);
  src_ed[std::make_pair(2,3)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  src_ed[std::make_pair(3,4)] = std::make_tuple(0, 3.14/2.0, 0, 1.0);
  src_ed[std::make_pair(4,0)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);
  src_ed[std::make_pair(1,4)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  src_ed[std::make_pair(1,3)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);

  src_ed[std::make_pair(1,0)] = src_ed[std::make_pair(0,1)];  // right roof
  src_ed[std::make_pair(2,1)] = src_ed[std::make_pair(1,2)];  // right wall
  src_ed[std::make_pair(3,2)] = src_ed[std::make_pair(2,3)];  // floor
  src_ed[std::make_pair(4,3)] = src_ed[std::make_pair(3,4)];  // left wall
  src_ed[std::make_pair(0,4)] = src_ed[std::make_pair(4,0)];  // left roof
  src_ed[std::make_pair(4,1)] = src_ed[std::make_pair(1,4)];  // ceiling
  src_ed[std::make_pair(3,1)] = src_ed[std::make_pair(1,3)];  // diagonal

  dst_ed[std::make_pair(0,1)] = std::make_tuple(3.14/4.0, 3.14/4.0, 0, 1.4);
  dst_ed[std::make_pair(1,2)] = std::make_tuple(0.0, 3.14/2.0, 0, 1.0);
  dst_ed[std::make_pair(2,4)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  dst_ed[std::make_pair(4,5)] = std::make_tuple(0, 3.14/2.0, 0, 1.0);
  dst_ed[std::make_pair(5,0)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);
  dst_ed[std::make_pair(1,5)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  dst_ed[std::make_pair(1,4)] = std::make_tuple(3.0 * 3.14/4.0, -3.14/4.0, 0, 1.4);

  dst_ed[std::make_pair(1,0)] = dst_ed[std::make_pair(0,1)];  // right roof
  dst_ed[std::make_pair(2,1)] = dst_ed[std::make_pair(1,2)];  // right wall
  dst_ed[std::make_pair(4,2)] = dst_ed[std::make_pair(2,4)];  // floor
  dst_ed[std::make_pair(5,4)] = dst_ed[std::make_pair(4,5)];  // left wall
  dst_ed[std::make_pair(0,5)] = dst_ed[std::make_pair(5,0)];  // left roof
  dst_ed[std::make_pair(5,1)] = dst_ed[std::make_pair(1,5)];  // ceiling
  dst_ed[std::make_pair(4,1)] = dst_ed[std::make_pair(1,4)];  // diagonal

  dst_ed[std::make_pair(2,3)] = std::make_tuple(3.14/2.0, 0, 0, 1.0);
  dst_ed[std::make_pair(3,2)] = dst_ed[std::make_pair(2,3)];  // tail
}

EdgeSimilarityMatrix *testmultiple_artificial_edgesimilarity()
{
  std::map<std::pair<KeyT, KeyT>, unsigned int> sourceEdgeIndex;
  std::map<std::pair<KeyT, KeyT>, unsigned int> destEdgeIndex;
  MatrixDouble esim_m(2*7, 2*8);

  sourceEdgeIndex[std::make_pair(0,1)] = 0;
  sourceEdgeIndex[std::make_pair(1,2)] = 2;
  sourceEdgeIndex[std::make_pair(2,3)] = 4;
  sourceEdgeIndex[std::make_pair(3,4)] = 6;
  sourceEdgeIndex[std::make_pair(4,0)] = 8;
  sourceEdgeIndex[std::make_pair(4,1)] = 10;
  sourceEdgeIndex[std::make_pair(1,3)] = 12;

  sourceEdgeIndex[std::make_pair(1,0)] = 1;
  sourceEdgeIndex[std::make_pair(2,1)] = 3;
  sourceEdgeIndex[std::make_pair(3,2)] = 5;
  sourceEdgeIndex[std::make_pair(4,3)] = 7;
  sourceEdgeIndex[std::make_pair(0,4)] = 9;
  sourceEdgeIndex[std::make_pair(1,4)] = 11;
  sourceEdgeIndex[std::make_pair(3,1)] = 13;

  destEdgeIndex[std::make_pair(0,1)] = 0;
  destEdgeIndex[std::make_pair(1,2)] = 2;
  destEdgeIndex[std::make_pair(2,4)] = 4;
  destEdgeIndex[std::make_pair(4,5)] = 6;
  destEdgeIndex[std::make_pair(5,0)] = 8;
  destEdgeIndex[std::make_pair(5,1)] = 10;
  destEdgeIndex[std::make_pair(1,4)] = 12;
  destEdgeIndex[std::make_pair(2,3)] = 14;

  destEdgeIndex[std::make_pair(1,0)] = 1;
  destEdgeIndex[std::make_pair(2,1)] = 3;
  destEdgeIndex[std::make_pair(4,2)] = 5;
  destEdgeIndex[std::make_pair(5,4)] = 7;
  destEdgeIndex[std::make_pair(0,5)] = 9;
  destEdgeIndex[std::make_pair(1,5)] = 11;
  destEdgeIndex[std::make_pair(4,1)] = 13;
  destEdgeIndex[std::make_pair(3,2)] = 15;

  esim_m.fill(1.0);   // all edges have same similarity, so all solutions are equally good

  //esim_m.topLeftCorner(14,14).setIdentity();

  return new EdgeSimilarityMatrix(sourceEdgeIndex, destEdgeIndex, esim_m);
}

// defined in printandcompare.cpp
double run_print_compare(int ng, int nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj, MatrixDouble const *humansolution);
//double run_print_compare(int ng, int nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj, std::vector<double> const *humansolution);

void test_multiple_optimal_solutions()
{
  int const ng = 5; // src graph has 5 nodes
  int const nh = 6; // dst graph has 6 nodes

  MatrixInt src_adj(ng,ng);
  MatrixInt dst_adj(nh,nh);
  testmultiple_fill_adjacency_matrices(src_adj, dst_adj);

  VertexSimilarityMatrix *vsim_ptr;
  EdgeSimilarityMatrix   *esim_ptr;

  bool use_artificial_matrices = true;

  if (!use_artificial_matrices)
  {
    ESFDescriptors  src_esf;
    ESFDescriptors  dst_esf;
    testmultiple_fill_esf_descr(src_esf, dst_esf);

    EdgeDescriptors src_ed;
    EdgeDescriptors dst_ed;
    testmultiple_fill_edge_descr(src_ed, dst_ed);

    vsim_ptr = new VertexSimilarityMatrix(src_esf, dst_esf);  // from ESF descriptors
    esim_ptr = new EdgeSimilarityMatrix(src_ed, dst_ed);      // from edge descriptors
  }
  else // if use_artificial_matrices
  {
    MatrixDouble vsim_1_m(ng,nh); // artificial vertex similarity matrix
    vsim_1_m.fill(1.0);           // all solutions will be equally good
    vsim_ptr = new VertexSimilarityMatrix(vsim_1_m);
    esim_ptr = testmultiple_artificial_edgesimilarity();
  }

  // output the chosen similarity matrices
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "Vertex similarity matrix:" << std::endl;
  std::cout << vsim_ptr->m << std::endl << std::endl;
  std::cout.unsetf(std::ios_base::floatfield);
  std::cout << std::setprecision(1);
  std::cout << "Edge similarity matrix:" << std::endl;
  std::cout << esim_ptr->m << std::endl << std::endl;
  std::cout << std::fixed << std::setprecision(2);

  // human-known graph-matching
  MatrixDouble human_x(ng, nh);
  human_x.topLeftCorner(3,3).setIdentity();     // g 0,1,2 <-> h 0,1,2
  human_x.bottomRightCorner(2,2).setIdentity(); // g 3,4   <-> h 4,5
  human_x.bottomLeftCorner(2,4).setZero();
  human_x.topRightCorner(3,3).setZero();

  run_print_compare(ng, nh, &vsim_ptr->m, esim_ptr, &src_adj, &dst_adj, &human_x);

  // // declare graph matching algorithm
  // GraphMatchingPath gm(&vsim_ptr->m, esim_ptr, &src_adj, &dst_adj);
  //
  // // declare stochastic matrix which will hold the solution graph matching
  // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> x(ng,nh);
  //
  // // initialise with trivial feasible solution
  // x << 0.16, 0.16, 0.16, 0.16, 0.16, 0.16,
  //      0.16, 0.16, 0.16, 0.16, 0.16, 0.16,
  //      0.16, 0.16, 0.16, 0.16, 0.16, 0.16,
  //      0.16, 0.16, 0.16, 0.16, 0.16, 0.16,
  //      0.16, 0.16, 0.16, 0.16, 0.16, 0.16;
  //
  // // solve
  // gm.frankWolfe(0.0, &x, &x);
  //
  // // output final solution
  // std::cout << std::endl << "Final solution:" << std::endl;
  // std::cout << x << std::endl;
  //
  // // output final score and compare with human score
  // std::cout << "Final similarity score:" << std::endl;
  // std::cout << "    " << gm.bilinear(x.data(), x.data()) << std::endl;
  // std::cout << std::endl << "Compare with human-known matching and score" << std::endl;
  // x.topLeftCorner(3,3).setIdentity();     // g 0,1,2 <-> h 0,1,2
  // x.bottomRightCorner(2,2).setIdentity(); // g 3,4   <-> h 4,5
  // x.bottomLeftCorner(2,4).setZero();
  // x.topRightCorner(3,3).setZero();
  // std::cout << "Human-known matching:" << std::endl;
  // std::cout << x << std::endl;
  // std::cout << "Human-known score:" << std::endl;
  // std::cout << "    " << gm.bilinear(x.data(), x.data()) << std::endl;

  // free memory
  delete vsim_ptr;
  delete esim_ptr;
}
