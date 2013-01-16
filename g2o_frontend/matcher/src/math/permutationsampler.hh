#ifndef STAT_HH
#define STAT_HH

#include <vector>

namespace AISNavigation {

struct PermutationSampler{
public:
  PermutationSampler(const std::vector<double>& v);
  ~PermutationSampler();
  int sampleWithRemoval(double d);
  int sample(double d);
  void recoverWeights();

  inline double getSum() const {return sum;}

private:
  struct PermutationSamplerNode{
    double leftSum;
    int info;
    PermutationSamplerNode *left, *right, *parent;
    inline bool isLeaf(){
      return left==0 && right==0;
    }
    inline bool isRoot(){
      return parent==0;
    }
  };

  PermutationSamplerNode* root;
  double sum;
  double totalSum;

  PermutationSamplerNode* binarySearch(double d);

  void remove (PermutationSamplerNode* n);

  double recoverWeights(PermutationSamplerNode* n);

  PermutationSamplerNode *  initializeTree(int level, PermutationSamplerNode* parent, 
					   const std::vector<double>& v, int& index);


  PermutationSamplerNode * initializeTree(int level, const std::vector<double>& v, int& index);

  void destroyTree(PermutationSamplerNode * n);
};


}; //namespace AISNavigation

#endif
