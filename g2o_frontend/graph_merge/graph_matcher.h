/*
struct RecursiveMatcher{
  Graph* g1, *g2;
  double visitDistance;


  std::set<Edge*> matches;
  std::set<Node*> visited1, visited2;

  double nodeDistance, nodeRotation;

  matchChildren(Node* n1, Node* n2) {
    Eigen::Isometry2d n2pose= n2->pose;
    const EdgeSet& n1Set = n1->edges();
    const EdgeSet& n2Set = n2->edges();
    for (EdgeSet::const_iterator it = n1Set.begin(); it!=n1Set.end(); it++) {
      Edge* e1=*it;
      Node* n1Other = (n1 == e1->from)? e1->to : e1->from;
      if (visited1.count(n1Other))
    continue;
      visited1.insert(n1Other);

      Node* bestN2 = 0;
      float bestChi = 1e9;

      Eigen::Isometry2d n1OtherInverse = n1->pose.inverse();
      for (EdgeSet::const_iterator it = n2Set.begin(); it!=n2Set.end(); it++) {
    Edge* e2=*it;
    Eigen::Isometry2d t = e2->transform;
    Node* n2Other = e2->to;
    if(n2 == e2->to){
      t = t.inverse();
      n2Other = e2->from;
    }
    if (visited2.count(n2Other))
      continue;
    n2Other.pose = n2.pose*t;

    Vector3d delta = t2v(n1OtherInverse*n2Other.pose);
    if (delta.squaredNorm()<bestChi){
      bestN2 = n2Other;
      bestChi = delta;
    }
      }
      if (bestN2)
    visited2.insert(bestN2);
      visited1.erase(n1Other);
    }
  }

}
*/
