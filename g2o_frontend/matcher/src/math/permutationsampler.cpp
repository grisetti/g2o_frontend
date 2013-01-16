#include <cmath>
#include <assert.h>
#include "permutationsampler.hh"
#include <iostream>

using namespace std;
namespace AISNavigation{

  PermutationSampler::PermutationSampler(const std::vector<double>& v){
    int level=(int)ceil(log2(v.size()))+1;
    int index=0;
    root=initializeTree(level, 0, v, index);
    totalSum=0;
    for (int i=0; i<(int)v.size(); i++){
      totalSum+=v[i];
    }
    recoverWeights(root);
    sum=totalSum;
  }

  PermutationSampler::~PermutationSampler(){
    destroyTree(root);
    sum=0;
    root=0;
  }

  int PermutationSampler::sampleWithRemoval(double d){
    PermutationSamplerNode * n=binarySearch(d);
    if (n){
      remove(n);
      return n->info;
    }
    return -1;
  }

  int PermutationSampler::sample(double d){
    PermutationSamplerNode * n=binarySearch(d);
    if (n){
      return n->info;
    }
    return -1;
  }

  void PermutationSampler::recoverWeights(){
    recoverWeights(root);
    sum=totalSum;
  }


  PermutationSampler::PermutationSamplerNode* PermutationSampler::binarySearch(double d) {
    PermutationSamplerNode* aux=root;
    while (aux && ! aux->isLeaf()){
      if (d<=aux->leftSum)
	aux=aux->left;
      else{
	d-=aux->leftSum;
	aux=aux->right;
      }
    }
    return aux;
  }

  void PermutationSampler::remove(PermutationSampler::PermutationSamplerNode* n){
    assert (n->isLeaf());
    double v=n->leftSum;
    PermutationSamplerNode* np=n;
    while (n!=root){
      n=n->parent;
      if (n->left==np){
	n->leftSum-=v;
      }
      np=n;
    }
    sum-=v;
  }

  double PermutationSampler::recoverWeights(PermutationSampler::PermutationSamplerNode* n) {
    if (n->isLeaf()){
      return n->leftSum;
    }
    if (!n)
      return 0;
    double leftSum  = recoverWeights(n->left);
    double rightSum = recoverWeights(n->right);
    n->leftSum=leftSum;
    return leftSum+rightSum;
  }

  PermutationSampler::PermutationSamplerNode * 
      PermutationSampler::initializeTree(int level, PermutationSampler::PermutationSamplerNode* parent, 
					 const std::vector<double>& v, int& index){
    if (level==0)
      return 0;

    PermutationSamplerNode* n=new PermutationSamplerNode;
    n->parent=parent;
    if (level==1){
      n->left=0;
      n->right=0;
      n->info=index;
      n->leftSum = (index<(int)v.size()) ? n->leftSum=v[index++] : 0;
    } else {
      n->left  = initializeTree (level-1, n, v, index);
      n->right = initializeTree (level-1, n, v, index);
    }
    return n;
  }

  void PermutationSampler::destroyTree(PermutationSampler::PermutationSamplerNode * n){
    if (!n)
      return;
    destroyTree(n->left);
    destroyTree(n->right);
    delete n;
  }


}; // namespace AISNavigation
