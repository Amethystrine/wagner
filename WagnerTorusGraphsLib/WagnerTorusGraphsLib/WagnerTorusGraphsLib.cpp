#include "WagnerTorusGraphsLib.h"

namespace WagnerTorusGraphs {

WagnerTorusGraphsLib::WagnerTorusGraphsLib() :
  diagverifyfunc_param_(nullptr),
  DiagonalVerifyFunc_(nullptr)
{
  NodeInfo defaultnode[kDefaultThreeNode];
  defaultnode[0].SetCoord(0, kCoordLimit*5);
  defaultnode[1].SetCoord(-kCoordLimit * 5, -kCoordLimit * 5);
  defaultnode[2].SetCoord(kCoordLimit * 5, -kCoordLimit * 5);

  for (int i = 0; i < kDefaultThreeNode; i++) {
    defaultnode[i].AddEdge((i + 1) % kDefaultThreeNode, kDefault, 0);
    defaultnode[i].AddEdge((i + 2) % kDefaultThreeNode, kDefault, 1);
    nodelist_vector_.push_back(defaultnode[i]);
  }
}
WagnerTorusGraphsLib::~WagnerTorusGraphsLib() {
  nodelist_vector_.clear();
  diagonallist_vector_.clear();
}

// 該当するNodeInfoを登録し、登録したNodeIndexを返す。他のNodeと座標が一致していた場合、そのノードのNodeIndexを返す。
NodeIndex WagnerTorusGraphsLib::RegistNode(NodeInfo& node) {
  // 所属位置を検索する
  NodeTriangle tri;
  FindTrianglePosition(node.xy_coord_, tri, GetLastNode());
  // 点が一致していたら登録せずに、一致している点を返す
  for (int i = 0; i < 3; ++i) {
    if (IsSameCoord(node.xy_coord_, nodelist_vector_[tri[i]].xy_coord_)) return tri[i];
  }
  if (tri[0] == tri[1] && tri[0] == tri[2]) return kUndefined;
  if (tri[0] == tri[1]) return DetermineNodeOnEdge(node, NodePair{ tri[1],tri[2] });
  if (tri[0] == tri[2]) return DetermineNodeOnEdge(node, NodePair{ tri[1],tri[2] });
  if (tri[1] == tri[2]) return DetermineNodeOnEdge(node, NodePair{ tri[0],tri[2] });
  return DetermineNodeOnTriangle(node, tri);
}

// nodeの接続情報をなくす。NodeIndexのズレを防止する為データは残る
// do_verifyをすると、diagonal transformationに関わった辺も候補に入れてdiagonal transformationする。ほかの変形候補も消化される。
bool WagnerTorusGraphsLib::DeleteNode(NodeIndex node, bool do_verify) {
  // 可能な限りdiagonal transformationをしていく
  if (HasUncuttableID(node)) return false;
  NodeInfo& info = nodelist_vector_[node];
  for (EdgeIndex i_edge = 0; i_edge < info.GetAccessSize(); i_edge++) {
    NodeQuadrangle quad;
    if (!MakeQuadrangle(node, i_edge, quad)) continue;
    if (!CanDiagonalTransformation(quad)) continue;
    if (DoQuadDiagonalTransformation(quad)) {
      if (do_verify) PushDiagonalListQuadrangle(quad);
      i_edge = -1;  //成功したら最初からもう一度調べる
    }
  }
  // 残りの点数に応じて最後の接続も切断する
  switch (info.GetAccessSize()) {
  case 3:
    if (do_verify) PushDiagonalListTriangle(&info.accessnodelist_vector_[0]);
    // そのまま全て切断する
    CutEdge(node, info.GetNode(2));
    CutEdge(node, info.GetNode(1));
    CutEdge(node, info.GetNode(0));
    break;
  case 4:
  {
    NodeQuadrangle erase_quad = { info.GetNode(0), info.GetNode(1), info.GetNode(2), info.GetNode(3) };
    if (do_verify) PushDiagonalListQuadrangle(erase_quad);
    // 0と2を接続させる。IDは0との接続を優先させる
    EdgeID CutID = info.GetID(0);
    EdgeIndex edge0 = nodelist_vector_[erase_quad[0]].GetEdge(node);
    EdgeIndex edge2 = nodelist_vector_[erase_quad[2]].GetEdge(node);
    nodelist_vector_[erase_quad[0]].accessnodelist_vector_[edge0] = erase_quad[2];
    nodelist_vector_[erase_quad[2]].accessnodelist_vector_[edge2] = erase_quad[0];
    nodelist_vector_[erase_quad[0]].accessidlist_vector_[edge0] = InvertedID(CutID);
    nodelist_vector_[erase_quad[2]].accessidlist_vector_[edge2] = CutID;
    // 1と3は切断してしまう
    CutEdge(node, info.GetNode(3));
    CutEdge(node, info.GetNode(1));
    // 接続を抹消
    info.accessnodelist_vector_.clear();
    info.accessidlist_vector_.clear();
    break;
  }
  default:
    return false;
  }
  if (do_verify) DoAllVerify();
  return true;
}

// node1からnode2を接続させる。bChainVeryfyをtrueにdiagonal transformationに関わった四角形を検証コンテナに入れて変形させる
int WagnerTorusGraphsLib::BridgeNode(NodeIndex node1, NodeIndex node2, bool do_chainverify, int regression_count) {
  // 初期の三点は対象外とする
  if (node1 < kDefaultThreeNode || node2 < kDefaultThreeNode || 
      node1 >= nodelist_vector_.size() || node2 >= nodelist_vector_.size() ||
      node1 == node2) {
    printf("Default Node can't use BridgeNode\n");
    return kUndefined;
  }
  // 接続確認できるまでdiagonal transformationを繰り返す
  bool transed_once_or_more = false;  //一度でもdiagonal transformationしたか記録
  NodeInfo& info1 = nodelist_vector_[node1];
  while (info1.GetEdge(node2) == kUndefined) {
    // node2に向けた方向を探す
    EdgeIndex edgeDirec = GetSandwichVectorEdge(node1, nodelist_vector_[node2].xy_coord_);
    // 該当する方向の二点をdiagonal transformationさせる
    NodeQuadrangle quad;
    if (!MakeQuadrangle(nodelist_vector_[node1].GetNode(edgeDirec), nodelist_vector_[node1].GetNode(edgeDirec + 1), quad)) {
      printf("Quad make failed\n");
      return kUndefined;
    }
    if (CanDiagonalTransformation(quad)) {
      DoQuadDiagonalTransformation(quad);
      if (do_chainverify) PushDiagonalListQuadrangle(quad);
      transed_once_or_more = true;
      continue;
    }
    // diagonal transformationできなかった場合
    else {
      int reentrant_vertex = QuadrantHasReentrant(quad);
      // 現在の対角線を結んでいる点のどちらかが凹角だった場合、そこから接続するよう再帰する
      if (reentrant_vertex == 0 || reentrant_vertex == 2) {
        int nResult = BridgeNode(quad[reentrant_vertex], node2, do_chainverify, regression_count + 1);
        if (nResult != kUndefined) {
          NodeTriangle tri0 = { quad[3], quad[0], quad[1] };
          NodeTriangle tri2 = { quad[1], quad[2], quad[3] };
          // 真っ直ぐにつながっている場合は、無理につながずにつながっているものとして返す
          if ((reentrant_vertex == 0 && IsStraight(tri0)) || (reentrant_vertex == 2 && IsStraight(tri2))) {
            return nResult + 1;
          }
          continue;
        }
        return kUndefined;
      }
      // 再帰していて一回以上diagonal transformationをしていたのであれば再帰前に戻る
      if (regression_count > 0 && transed_once_or_more) return true;
      // 再帰していない場合、diagonal transformationを行っていない場合は不適切
      printf("ConnectedFailed : Other Edge Blocked?\n");
      return kUndefined;
    }
  }
  if (do_chainverify && regression_count == 0) {
    // IDを保存して一次的に固定エッジにし、diagonal transformation後戻す。
    EdgeID tmpID = info1.GetID(info1.GetEdge(node2));
    SetEdgeID(node1, node2, kUncuttable);
    DoAllVerify();
    SetEdgeID(node1, node2, tmpID);
  }
  return 0;
}

// エッジにIDを登録する
bool WagnerTorusGraphsLib::SetEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID) {
  // 初期の三点は対象外
  if (node1 < kDefaultThreeNode && node2 < kDefaultThreeNode ||
    node1 >= nodelist_vector_.size() || node2 >= nodelist_vector_.size()) {
    return false;
  }
  EdgeIndex edge1 = nodelist_vector_[node1].GetEdge(node2);
  EdgeIndex edge2 = nodelist_vector_[node2].GetEdge(node1);
  if (edge1 == kUndefined || edge2 == kUndefined) return false;
  nodelist_vector_[node1].accessidlist_vector_[edge1] = ID;
  nodelist_vector_[node2].accessidlist_vector_[edge2] = InvertedID(ID);
  return true;
}
bool WagnerTorusGraphsLib::SetAllEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID) {
  // 初期の三点は対象外
  if (node1 < kDefaultThreeNode && node2 < kDefaultThreeNode ||
      node1 >= nodelist_vector_.size() || node2 >= nodelist_vector_.size()) {
    return false;
  }
  NodeInfo& info1 = nodelist_vector_[node1];
  if (info1.GetEdge(node2) == kUndefined) {
    // node2に向けた方向を探す
    EdgeIndex edgeDirec = GetSandwichVectorEdge(node1, nodelist_vector_[node2].xy_coord_);
    NodeIndex nodeDirec = info1.accessnodelist_vector_[edgeDirec];
    // ここまでを接続し、そこから再度接続する
    if (!SetEdgeID(node1, nodeDirec, ID)) return false;
    return SetAllEdgeID(nodeDirec, node2, ID);
  }
  return SetEdgeID(node1, node2, ID);
}

// 指定エッジを無条件にdiagonal transformationをさせる. true:変形成功　false:変形失敗
bool WagnerTorusGraphsLib::DiagonalTransformation(NodeIndex node, EdgeIndex edge) {
  NodeQuadrangle quad;
  MakeQuadrangle(node, edge, quad);
  if (!CanDiagonalTransformation(quad)) return false;
  return DoQuadDiagonalTransformation(quad);
}
bool WagnerTorusGraphsLib::DiagonalTransformation(NodeIndex node1, NodeIndex node2) {
  EdgeIndex edge1 = nodelist_vector_[node1].GetEdge(node2);
  if (edge1 == kUndefined)return false;
  return DiagonalTransformation(node1, edge1);
}

// 全部の辺をdiagonal transformation対象にして実行
void WagnerTorusGraphsLib::AllVerifyDiagonalTransformation() {
  AddAllEdgeToDiagonalList();
  DoAllVerify();
}

// データ出力
void WagnerTorusGraphsLib::GetNode(void* param, GetNodeFunc func) {
  for (NodeIndex i_node = kDefaultThreeNode; i_node < nodelist_vector_.size(); i_node++) {
    if (!IsDeleted(i_node)) {
      func(param, nodelist_vector_[i_node].xy_coord_, nodelist_vector_[i_node].infoparam_);
    }
  }
}
void WagnerTorusGraphsLib::GetEdge(void* param, GetEdgeFunc func) {
  for (NodeIndex i_node = kDefaultThreeNode; i_node < nodelist_vector_.size(); i_node++) {
    EdgeIndex list_size = nodelist_vector_[i_node].GetAccessSize();
    for (EdgeIndex i_edge = 0; i_edge < list_size; i_edge++) {
      NodeIndex j_node = GetAccessNode(i_node, i_edge);
      if (i_node < j_node) {
        Vec2d coord[2] = { {nodelist_vector_[i_node].xy_coord_[X], nodelist_vector_[i_node].xy_coord_[Y]},
                        {nodelist_vector_[j_node].xy_coord_[X], nodelist_vector_[j_node].xy_coord_[Y]} };
        void* infoparam[2] = { nodelist_vector_[i_node].infoparam_, nodelist_vector_[j_node].infoparam_ };
        func(param, coord, infoparam);
      }
    }
  }
}
void WagnerTorusGraphsLib::GetTriangle(void* param, GetTriangleFunc func) {
  for (NodeIndex i_node = kDefaultThreeNode; i_node < nodelist_vector_.size(); i_node++) {
    EdgeIndex list_size = nodelist_vector_[i_node].GetAccessSize();
    for (EdgeIndex i_edge = 0; i_edge < list_size; i_edge++) {
      NodeIndex j_node = nodelist_vector_[i_node].GetNode(i_edge);
      NodeIndex k_node = nodelist_vector_[i_node].GetNode(i_edge + 1);
      if (i_node < j_node && i_node < k_node) {
        Vec2d coord[3] = { {nodelist_vector_[i_node].xy_coord_[X], nodelist_vector_[i_node].xy_coord_[Y]},
                        {nodelist_vector_[j_node].xy_coord_[X], nodelist_vector_[j_node].xy_coord_[Y]},
                        {nodelist_vector_[k_node].xy_coord_[X], nodelist_vector_[k_node].xy_coord_[Y]} };
        void* infoparam[3] = { nodelist_vector_[i_node].infoparam_, nodelist_vector_[j_node].infoparam_, nodelist_vector_[k_node].infoparam_ };
        func(param, coord, infoparam);
      }
    }
  }
}

// IDが向きによって変化する場合はここで反転させる
EdgeID WagnerTorusGraphsLib::InvertedID(EdgeID ID) {
  for (const auto& invertID : kInvertIDList) {
    if (ID == invertID[0]) return invertID[1];
    if (ID == invertID[1]) return invertID[0];
  }
  return ID;
}

//Protected

// 現段階の全てのエッジを変形候補リストに格納する
void WagnerTorusGraphsLib::AddAllEdgeToDiagonalList() {
  for (NodeIndex i_node = 0; i_node < nodelist_vector_.size(); i_node++) {
    EdgeIndex list_size = nodelist_vector_[i_node].GetAccessSize();
    for (EdgeIndex i_edge = 0; i_edge < list_size; i_edge++) {
      NodeIndex j_node = GetAccessNode(i_node, i_edge);
      if (i_node < j_node) {
        NodePair diagonal{ i_node, j_node };
        diagonallist_vector_.push_back(diagonal);
      }
    }
  }
}

// 変形候補リスト内のdiagonal transformation検証、及び変形を全て調べるまで行う
// do_chainverifyがtrueの場合、diagonal transformationに使用した四辺も追加でリスト内に入れる
void WagnerTorusGraphsLib::DoAllVerify(bool do_chainverify) {
  int trans_count = 0;
  
  const int max_trans_count = kSqrtMaxTransCountStopper > nodelist_vector_.size() ?
                              nodelist_vector_.size() * (nodelist_vector_.size() - 1) :
                              kSqrtMaxTransCountStopper * kSqrtMaxTransCountStopper;
  // リストの末尾まで調べる
  for (int i_diagonal = 0; i_diagonal < diagonallist_vector_.size(); i_diagonal++) {
    // 対角線がまだ存在しているか
    NodeQuadrangle trans_quad;
    NodePair nodepair = diagonallist_vector_[i_diagonal];
    if (!MakeQuadrangle(nodepair.nodeA, nodepair.nodeB, trans_quad)) continue;
    // diagonal transformationの共通判定を行う
    TransJudge must_trans_judge = JudgeDiagonalTransformation(trans_quad);
    if (must_trans_judge == kMustNotTrans) continue;
    if (must_trans_judge == kVerify) {
      // diagonal transformationするか判定する
      Vec2d trans_quad_coord[4];
      void* ainfoparam_trans_quad[4];
      for (int i = 0; i < 4; i++) {
        trans_quad_coord[i][X] = nodelist_vector_[trans_quad[i]].xy_coord_[X];
        trans_quad_coord[i][Y] = nodelist_vector_[trans_quad[i]].xy_coord_[Y];
        ainfoparam_trans_quad[i] = nodelist_vector_[trans_quad[i]].infoparam_;
      }
      bool should_trans;
      try {
        // 登録関数の使用
        if (DiagonalVerifyFunc_ == nullptr) throw 0;
        should_trans = DiagonalVerifyFunc_(diagverifyfunc_param_, trans_quad_coord, ainfoparam_trans_quad);
      }
      catch (...) {
        // 既存関数の使用
        if (DiagonalVerifyFunc_ != nullptr) printf("DiagonalVerifyFunc has error.\n");
        should_trans = DefaultVerify(trans_quad_coord);
      }
      if (!should_trans) continue;
    }
    // 実行
    DoQuadDiagonalTransformation(trans_quad);
    if (do_chainverify) PushDiagonalListQuadrangle(trans_quad);
    if (++trans_count >= max_trans_count) {
      printf("DiagonalTransformation Count Over : Check DiagonalVerifyFunc is a condition for solution\n");
      break;
    }
  }
  diagonallist_vector_.clear();
}

// diagonal transformation検証の関数が登録されていなかった場合に、diagonal transformationすべきか判定するデフォルト関数
// 二次元座標上で、四角形の対角の和が180度以上になる方に対角線を張る
bool WagnerTorusGraphsLib::DefaultVerify(Vec2d coord[4]) {
  // 現在の対角線0-2の内積取得
  Vec2d vec[4];
  for (int i = 0; i < 4; i++) {
    vec[i][X] = coord[(i + 1) % 4][X] - coord[i][X];
    vec[i][Y] = coord[(i + 1) % 4][Y] - coord[i][Y];
  }
  double inner0 = -vec[0][X] * vec[3][X] - vec[0][Y] * vec[3][Y];
  double inner2 = -vec[1][X] * vec[2][X] - vec[1][Y] * vec[2][Y];
  // どっちも鋭角なら0-2に対角線は張るべきでない
  if (inner0 > 0 && inner2 > 0) return true;
  // どっちも鈍角なら0-2に対角線は張ったままが良い
  if (inner0 < 0 && inner2 < 0) return false;
  // 混じった場合、cosを求めて足して負の場合は、二角の合計が180度以上になる
  double cos0 = inner0 / sqrt((vec[0][X] * vec[0][X] + vec[0][Y] * vec[0][Y]) * (vec[3][X] * vec[3][X] + vec[3][Y] * vec[3][Y]));
  double cos2 = inner2 / sqrt((vec[2][X] * vec[2][X] + vec[2][Y] * vec[2][Y]) * (vec[1][X] * vec[1][X] + vec[1][Y] * vec[1][Y]));
  return cos0 + cos2 > 1e-20;
}

// 該当するエッジを対角線とした四角形の番号を取得する
// quadのルールは0を入力されたnodeとし、反時計回り順に並べる。対角線は0-2で生成
bool WagnerTorusGraphsLib::MakeQuadrangle(NodeIndex node, EdgeIndex edge, NodeQuadrangle quad) {
  quad[0] = node;
  quad[1] = nodelist_vector_[node].GetNode(edge - 1);
  quad[2] = nodelist_vector_[node].GetNode(edge + 0);
  quad[3] = nodelist_vector_[node].GetNode(edge + 1);
  return true;
}
bool WagnerTorusGraphsLib::MakeQuadrangle(NodeIndex node1, NodeIndex node2, NodeQuadrangle quad) {
  EdgeIndex edge1 = nodelist_vector_[node1].GetEdge(node2);
  if (edge1 == kUndefined) return false;
  return MakeQuadrangle(node1, edge1, quad);
}

// そもそもdiagonal transformationできるか
bool WagnerTorusGraphsLib::CanDiagonalTransformation(NodeQuadrangle quad) {
  // 現在の対角線0-2のどちらかが凹角
  int reentrant_vertex = QuadrantHasReentrant(quad);
  if (reentrant_vertex == 0 || reentrant_vertex == 2) return false;
  // 現在の対角線0-2が切断不可能IDである
  NodeInfo & info0 = nodelist_vector_[quad[0]];
  EdgeID ID = info0.GetID(info0.GetEdge(quad[2]));
  if (IsUncuttableID(ID)) return false;
  return true;
}
// diagonal transformationをしなくてはならないか、してはいけないか、Verify関数か判定する
TransJudge WagnerTorusGraphsLib::JudgeDiagonalTransformation(NodeQuadrangle quad) {
  if (!CanDiagonalTransformation(quad)) return kMustNotTrans;
  if (quad[1] < kDefaultThreeNode || quad[3] < kDefaultThreeNode) return kMustNotTrans;
  if (quad[0] < kDefaultThreeNode || quad[2] < kDefaultThreeNode) return kMustTrans;
  return kVerify;
}

// MakeQuadrangleで作成された正常なQuadであることを前提にdiagonal transformation、0-2の切断と1-3の接続を行う
bool WagnerTorusGraphsLib::DoQuadDiagonalTransformation(NodeQuadrangle quad) {
  EdgeIndex edge[4];
  // 0-2の切断
  edge[0] = nodelist_vector_[quad[0]].GetEdge(quad[2]);
  edge[2] = nodelist_vector_[quad[2]].GetEdge(quad[0]);
  if (edge[0] == kUndefined || edge[2] == kUndefined) return false;
  nodelist_vector_[quad[0]].EraseEdge(edge[0]);
  nodelist_vector_[quad[2]].EraseEdge(edge[2]);
  // 1-3の接続
  edge[1] = nodelist_vector_[quad[1]].GetEdge(quad[2]);
  edge[3] = nodelist_vector_[quad[3]].GetEdge(quad[0]);
  if (edge[1] == kUndefined || edge[3] == kUndefined) return false;
  nodelist_vector_[quad[1]].AddEdge(quad[3], kPlane, edge[1] + 1);
  nodelist_vector_[quad[3]].AddEdge(quad[1], kPlane, edge[3] + 1);
  return true;
}

// nodeの二次元座標からdestination_coordまでのベクトルは、どのエッジとエッジに挟まれているか返す
EdgeIndex WagnerTorusGraphsLib::GetSandwichVectorEdge(NodeInfo& node, Vec2d destination_coord) {
  if (IsSameCoord(node.xy_coord_, destination_coord)) { return 0; }
  EdgeIndex accesslist_size = node.GetAccessSize();
  for (EdgeIndex i_edge = 0; i_edge < accesslist_size; i_edge++) {
    NodeIndex node1 = node.GetNode(i_edge);
    NodeIndex node2 = node.GetNode(i_edge + 1);
    bool node1_is_anticlock = IsLevorotation(node.xy_coord_, nodelist_vector_[node1].xy_coord_, destination_coord);
    bool node2_is_anticlock = IsLevorotation(node.xy_coord_, nodelist_vector_[node2].xy_coord_, destination_coord);
    if (node1_is_anticlock && !node2_is_anticlock) return i_edge;
  }
  printf("DirectionEdge Failed\n");
  return kUndefined;
}
EdgeIndex WagnerTorusGraphsLib::GetSandwichVectorEdge(NodeIndex node, Vec2d destination_coord) {
  return GetSandwichVectorEdge(nodelist_vector_[node], destination_coord);
}

// coord1→coord2→coord3への経路で反時計回りに曲がっているか判定。平行は直進はtrue、逆向きはfalseとする
bool WagnerTorusGraphsLib::IsLevorotation(Vec2d coord1, Vec2d coord2, Vec2d coord3) {
  double cross_product = coord2[X] * coord3[Y] - coord2[X] * coord1[Y] - coord1[X] * coord3[Y]
    - coord2[Y] * coord3[X] + coord2[Y] * coord1[X] + coord1[Y] * coord3[X];
  if (cross_product == 0) return (coord2[X] - coord1[X]) * (coord3[X] - coord1[X]) + (coord2[Y] - coord1[Y]) * (coord3[Y] - coord1[Y]) > 0;
  return cross_product > 0;
}
bool WagnerTorusGraphsLib::IsLevorotation(NodeTriangle tri) {
  return IsLevorotation(nodelist_vector_[tri[0]].xy_coord_, nodelist_vector_[tri[1]].xy_coord_, nodelist_vector_[tri[2]].xy_coord_);
}
bool WagnerTorusGraphsLib::IsStraight(Vec2d coord1, Vec2d coord2, Vec2d coord3) {
  return (coord2[X] * coord3[Y] - coord2[X] * coord1[Y] - coord1[X] * coord3[Y]
    - coord2[Y] * coord3[X] + coord2[Y] * coord1[X] + coord1[Y] * coord3[X]) == 0;
}
bool WagnerTorusGraphsLib::IsStraight(NodeTriangle tri) {
  return IsStraight(nodelist_vector_[tri[0]].xy_coord_, nodelist_vector_[tri[1]].xy_coord_, nodelist_vector_[tri[2]].xy_coord_);
}

//反時計回り 直線または凹角がないか探索し、あった場合はその角のイテレータを返す
int WagnerTorusGraphsLib::QuadrantHasReentrant(NodeQuadrangle quad) {
  for (int iVertex = 0; iVertex < 4; iVertex++) {
    NodeTriangle tri = { quad[(iVertex + 3) % 4],quad[iVertex],quad[(iVertex + 1) % 4] };
    if (!IsLevorotation(tri) || IsStraight(tri)) return iVertex;
  }
  return kUndefined;
}

int WagnerTorusGraphsLib::FindTrianglePosition(Vec2d coord, NodeTriangle tri, NodeIndex start_node) {
  // 失敗時処理
  auto not_found = [&] {
    tri[0] = kUndefined;
    tri[1] = kUndefined;
    tri[2] = kUndefined;
  };
  // 最大四点を保存する探索を行う
  NodeQuadrangle circling_cuad = { start_node, kUndefined, kUndefined, kUndefined };
  NodeIndex find_count = 0;
  const NodeIndex max_find_count = NodeIndex(nodelist_vector_.size());
  char vertex = 0;
  while (find_count++ < max_find_count) {
    // 現在のvertexから探索座標に向く接続を調べる
    EdgeIndex approaches_edge = GetSandwichVectorEdge(circling_cuad[vertex], coord);
    NodeIndex next_node = GetAccessNode(circling_cuad[vertex], approaches_edge);
    // 登録したvertexが同じ箇所を回っていないか吟味
    {
      if (IsSameCoord(coord, nodelist_vector_[next_node].xy_coord_)) {
        tri[0] = next_node;
        tri[1] = next_node;
        tri[2] = next_node;
        return 1;
      }
      // 3つ前と同じ点に向かっていたら、四角形を回っている。対角線を取る。
      if (next_node == circling_cuad[(vertex + 1) % 4]) {
        if (nodelist_vector_[circling_cuad[0]].GetEdge(circling_cuad[2]) != kUndefined) {
          tri[0] = circling_cuad[0];
          tri[1] = circling_cuad[2];
          tri[2] = circling_cuad[2];
          return 2;
        }
        if (nodelist_vector_[circling_cuad[1]].GetEdge(circling_cuad[3]) != kUndefined) {
          tri[0] = circling_cuad[1];
          tri[1] = circling_cuad[3];
          tri[2] = circling_cuad[3];
          return 2;
        }
        not_found();
        return 0;
      }
      // 2つ前と同じ点に向かっていたら三角形を回っている。三角形を取る。
      if (next_node == circling_cuad[(vertex + 2) % 4]) {
        tri[0] = circling_cuad[(vertex + 2) % 4];
        tri[1] = circling_cuad[(vertex + 3) % 4];
        tri[2] = circling_cuad[(vertex + 4) % 4];
        return 3;
      }
      // 1つ前と同じ点に向かっていたら、一辺を回っている。そこを取る。
      if (next_node == circling_cuad[(vertex + 3) % 4]) {
        tri[0] = circling_cuad[(vertex + 4) % 4];
        tri[1] = circling_cuad[(vertex + 3) % 4];
        tri[2] = circling_cuad[(vertex + 3) % 4];
        return 2;
      }
      // 一つ前と同じことは通常あり得ない
      if (next_node == circling_cuad[(vertex + 4) % 4]) {
        not_found();
        return 0;
      }
    }
    // 周回をなかった場合は、circling_cuadの値を更新して次の点を調べる
    vertex = (vertex + 1) % 4;
    circling_cuad[vertex] = next_node;
  }
  not_found();
  return 0;
}

// 指定された三角形上で点の登録を確定し、確定した点番号を返す。必要ならdiagonal transformation判定を行う。
NodeIndex WagnerTorusGraphsLib::DetermineNodeOnTriangle(NodeInfo& node, NodeTriangle tri, bool do_chainverify) {
  EdgeID ID = GetTriangleEdgeID(tri);
  NodeIndex num_newnode = nodelist_vector_.size();
  for (int i = 0; i < 3; i++) {
    // 新点から三角形側へ接続
    node.accessnodelist_vector_.push_back(tri[i]);
    node.accessidlist_vector_.push_back(InvertedID(ID));
    // 三角形側から新点へ接続
    NodeInfo& info = nodelist_vector_[tri[i]];
    EdgeIndex edge = info.GetEdge(tri[(i + 1) % 3]);
    info.AddEdge(num_newnode, ID, edge+1);
  }
  // 新点登録
  nodelist_vector_.push_back(node);
  // 三角形周りの整理
  if (do_chainverify) {
    PushDiagonalListTriangle(tri);
    DoAllVerify();
  }
  return num_newnode;
}

// 指定された辺上で点の登録を確定し、確定した点番号を返す。必要ならdiagonal transformation判定を行う。
NodeIndex WagnerTorusGraphsLib::DetermineNodeOnEdge(NodeInfo& node, NodePair line, bool do_chainverify) {
  NodeQuadrangle quad;
  MakeQuadrangle(line.nodeA, line.nodeB, quad);
  EdgeID ID = GetTriangleEdgeID(quad);
  NodeIndex num_newnode = nodelist_vector_.size();
  for (int i = 0; i < 4; i++) {
    // 新点から四角形側へ接続
    node.accessnodelist_vector_.push_back(quad[i]);
    node.accessidlist_vector_.push_back(InvertedID(ID));
    // 四角形側から新点へ接続
    if (i == 0 || i == 2) {
      // 対角線側は切断して情報を変える
      NodeInfo& info = nodelist_vector_[quad[i]];
      EdgeIndex edge = nodelist_vector_[quad[i]].GetEdge(quad[(i + 2) % 4]);
      info.accessnodelist_vector_[edge] = num_newnode;
      info.accessidlist_vector_[edge] = ID;
    }
    else {
      // 対角線でないほうは追加
      NodeInfo& info = nodelist_vector_[quad[i]];
      EdgeIndex edge = info.GetEdge(quad[(i + 1) % 4]);
      info.AddEdge(num_newnode, ID, edge+1);
    }
  }
  // 新点登録
  nodelist_vector_.push_back(node);
  // 三角形周りの整理
  if (do_chainverify) {
    PushDiagonalListQuadrangle(quad);
    DoAllVerify();
  }
  return num_newnode;
}

// 二点を切る
bool WagnerTorusGraphsLib::CutEdge(NodeIndex node1, NodeIndex node2) {
  NodeInfo& info1 = nodelist_vector_[node1];
  NodeInfo& info2 = nodelist_vector_[node2];
  EdgeIndex edge1 = info1.GetEdge(node2);
  EdgeIndex edge2 = info2.GetEdge(node1);
  if (edge1 == kUndefined || edge2 == kUndefined) return false;
  if (IsUncuttableID(info1.GetID(edge1))) return false;
  info1.EraseEdge(edge1);
  info2.EraseEdge(edge2);
  return true;
}

// kSameLocationDistに従い同一点か判定する
bool WagnerTorusGraphsLib::IsSameCoord(Vec2d coord1, Vec2d coord2) {
  return abs(coord1[X] - coord2[X]) < kSameLocationDist && abs(coord1[Y] - coord2[Y]) < kSameLocationDist;
}
bool WagnerTorusGraphsLib::IsSameNode(NodeIndex node1, NodeIndex node2) {
  return IsSameCoord(nodelist_vector_[node1].xy_coord_, nodelist_vector_[node2].xy_coord_);
}

// IDが切り取れるか否か（kUncuttableIDListを参照）
bool WagnerTorusGraphsLib::IsUncuttableID(EdgeID ID) {
  for (const auto& uncuttableID : kUncuttableIDList) {
    if ( ID == uncuttableID ) return true;
  }
  return false;
}
bool WagnerTorusGraphsLib::HasUncuttableID(NodeIndex node) {
  NodeInfo& info = nodelist_vector_[node];
  for (EdgeIndex i_edge = 0; i_edge < info.GetAccessSize(); i_edge++) {
    if (IsUncuttableID(info.GetID(i_edge))) return true;
  }
  return false;
}

// 三角形に追加するエッジの属性を返す。将来拡張用
EdgeID WagnerTorusGraphsLib::GetTriangleEdgeID(NodeTriangle tri) {
  return kPlane;
}

} //namespace

