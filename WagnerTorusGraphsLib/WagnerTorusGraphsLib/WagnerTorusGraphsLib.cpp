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

// �Y������NodeInfo��o�^���A�o�^����NodeIndex��Ԃ��B����Node�ƍ��W����v���Ă����ꍇ�A���̃m�[�h��NodeIndex��Ԃ��B
NodeIndex WagnerTorusGraphsLib::RegistNode(NodeInfo& node) {
  // �����ʒu����������
  NodeTriangle tri;
  FindTrianglePosition(node.xy_coord_, tri, GetLastNode());
  // �_����v���Ă�����o�^�����ɁA��v���Ă���_��Ԃ�
  for (int i = 0; i < 3; ++i) {
    if (IsSameCoord(node.xy_coord_, nodelist_vector_[tri[i]].xy_coord_)) return tri[i];
  }
  if (tri[0] == tri[1] && tri[0] == tri[2]) return kUndefined;
  if (tri[0] == tri[1]) return DetermineNodeOnEdge(node, NodePair{ tri[1],tri[2] });
  if (tri[0] == tri[2]) return DetermineNodeOnEdge(node, NodePair{ tri[1],tri[2] });
  if (tri[1] == tri[2]) return DetermineNodeOnEdge(node, NodePair{ tri[0],tri[2] });
  return DetermineNodeOnTriangle(node, tri);
}

// node�̐ڑ������Ȃ����BNodeIndex�̃Y����h�~����׃f�[�^�͎c��
// do_verify������ƁAdiagonal transformation�Ɋւ�����ӂ����ɓ����diagonal transformation����B�ق��̕ό`�������������B
bool WagnerTorusGraphsLib::DeleteNode(NodeIndex node, bool do_verify) {
  // �\�Ȍ���diagonal transformation�����Ă���
  if (HasUncuttableID(node)) return false;
  NodeInfo& info = nodelist_vector_[node];
  for (EdgeIndex i_edge = 0; i_edge < info.GetAccessSize(); i_edge++) {
    NodeQuadrangle quad;
    if (!MakeQuadrangle(node, i_edge, quad)) continue;
    if (!CanDiagonalTransformation(quad)) continue;
    if (DoQuadDiagonalTransformation(quad)) {
      if (do_verify) PushDiagonalListQuadrangle(quad);
      i_edge = -1;  //����������ŏ����������x���ׂ�
    }
  }
  // �c��̓_���ɉ����čŌ�̐ڑ����ؒf����
  switch (info.GetAccessSize()) {
  case 3:
    if (do_verify) PushDiagonalListTriangle(&info.accessnodelist_vector_[0]);
    // ���̂܂ܑS�Đؒf����
    CutEdge(node, info.GetNode(2));
    CutEdge(node, info.GetNode(1));
    CutEdge(node, info.GetNode(0));
    break;
  case 4:
  {
    NodeQuadrangle erase_quad = { info.GetNode(0), info.GetNode(1), info.GetNode(2), info.GetNode(3) };
    if (do_verify) PushDiagonalListQuadrangle(erase_quad);
    // 0��2��ڑ�������BID��0�Ƃ̐ڑ���D�悳����
    EdgeID CutID = info.GetID(0);
    EdgeIndex edge0 = nodelist_vector_[erase_quad[0]].GetEdge(node);
    EdgeIndex edge2 = nodelist_vector_[erase_quad[2]].GetEdge(node);
    nodelist_vector_[erase_quad[0]].accessnodelist_vector_[edge0] = erase_quad[2];
    nodelist_vector_[erase_quad[2]].accessnodelist_vector_[edge2] = erase_quad[0];
    nodelist_vector_[erase_quad[0]].accessidlist_vector_[edge0] = InvertedID(CutID);
    nodelist_vector_[erase_quad[2]].accessidlist_vector_[edge2] = CutID;
    // 1��3�͐ؒf���Ă��܂�
    CutEdge(node, info.GetNode(3));
    CutEdge(node, info.GetNode(1));
    // �ڑ��𖕏�
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

// node1����node2��ڑ�������BbChainVeryfy��true��diagonal transformation�Ɋւ�����l�p�`�����؃R���e�i�ɓ���ĕό`������
int WagnerTorusGraphsLib::BridgeNode(NodeIndex node1, NodeIndex node2, bool do_chainverify, int regression_count) {
  // �����̎O�_�͑ΏۊO�Ƃ���
  if (node1 < kDefaultThreeNode || node2 < kDefaultThreeNode || 
      node1 >= nodelist_vector_.size() || node2 >= nodelist_vector_.size() ||
      node1 == node2) {
    printf("Default Node can't use BridgeNode\n");
    return kUndefined;
  }
  // �ڑ��m�F�ł���܂�diagonal transformation���J��Ԃ�
  bool transed_once_or_more = false;  //��x�ł�diagonal transformation�������L�^
  NodeInfo& info1 = nodelist_vector_[node1];
  while (info1.GetEdge(node2) == kUndefined) {
    // node2�Ɍ�����������T��
    EdgeIndex edgeDirec = GetSandwichVectorEdge(node1, nodelist_vector_[node2].xy_coord_);
    // �Y����������̓�_��diagonal transformation������
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
    // diagonal transformation�ł��Ȃ������ꍇ
    else {
      int reentrant_vertex = QuadrantHasReentrant(quad);
      // ���݂̑Ίp��������ł���_�̂ǂ��炩�����p�������ꍇ�A��������ڑ�����悤�ċA����
      if (reentrant_vertex == 0 || reentrant_vertex == 2) {
        int nResult = BridgeNode(quad[reentrant_vertex], node2, do_chainverify, regression_count + 1);
        if (nResult != kUndefined) {
          NodeTriangle tri0 = { quad[3], quad[0], quad[1] };
          NodeTriangle tri2 = { quad[1], quad[2], quad[3] };
          // �^�������ɂȂ����Ă���ꍇ�́A�����ɂȂ����ɂȂ����Ă�����̂Ƃ��ĕԂ�
          if ((reentrant_vertex == 0 && IsStraight(tri0)) || (reentrant_vertex == 2 && IsStraight(tri2))) {
            return nResult + 1;
          }
          continue;
        }
        return kUndefined;
      }
      // �ċA���Ă��Ĉ��ȏ�diagonal transformation�����Ă����̂ł���΍ċA�O�ɖ߂�
      if (regression_count > 0 && transed_once_or_more) return true;
      // �ċA���Ă��Ȃ��ꍇ�Adiagonal transformation���s���Ă��Ȃ��ꍇ�͕s�K��
      printf("ConnectedFailed : Other Edge Blocked?\n");
      return kUndefined;
    }
  }
  if (do_chainverify && regression_count == 0) {
    // ID��ۑ����Ĉꎟ�I�ɌŒ�G�b�W�ɂ��Adiagonal transformation��߂��B
    EdgeID tmpID = info1.GetID(info1.GetEdge(node2));
    SetEdgeID(node1, node2, kUncuttable);
    DoAllVerify();
    SetEdgeID(node1, node2, tmpID);
  }
  return 0;
}

// �G�b�W��ID��o�^����
bool WagnerTorusGraphsLib::SetEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID) {
  // �����̎O�_�͑ΏۊO
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
  // �����̎O�_�͑ΏۊO
  if (node1 < kDefaultThreeNode && node2 < kDefaultThreeNode ||
      node1 >= nodelist_vector_.size() || node2 >= nodelist_vector_.size()) {
    return false;
  }
  NodeInfo& info1 = nodelist_vector_[node1];
  if (info1.GetEdge(node2) == kUndefined) {
    // node2�Ɍ�����������T��
    EdgeIndex edgeDirec = GetSandwichVectorEdge(node1, nodelist_vector_[node2].xy_coord_);
    NodeIndex nodeDirec = info1.accessnodelist_vector_[edgeDirec];
    // �����܂ł�ڑ����A��������ēx�ڑ�����
    if (!SetEdgeID(node1, nodeDirec, ID)) return false;
    return SetAllEdgeID(nodeDirec, node2, ID);
  }
  return SetEdgeID(node1, node2, ID);
}

// �w��G�b�W�𖳏�����diagonal transformation��������. true:�ό`�����@false:�ό`���s
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

// �S���̕ӂ�diagonal transformation�Ώۂɂ��Ď��s
void WagnerTorusGraphsLib::AllVerifyDiagonalTransformation() {
  AddAllEdgeToDiagonalList();
  DoAllVerify();
}

// �f�[�^�o��
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

// ID�������ɂ���ĕω�����ꍇ�͂����Ŕ��]������
EdgeID WagnerTorusGraphsLib::InvertedID(EdgeID ID) {
  for (const auto& invertID : kInvertIDList) {
    if (ID == invertID[0]) return invertID[1];
    if (ID == invertID[1]) return invertID[0];
  }
  return ID;
}

//Protected

// ���i�K�̑S�ẴG�b�W��ό`��⃊�X�g�Ɋi�[����
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

// �ό`��⃊�X�g����diagonal transformation���؁A�y�ѕό`��S�Ē��ׂ�܂ōs��
// do_chainverify��true�̏ꍇ�Adiagonal transformation�Ɏg�p�����l�ӂ��ǉ��Ń��X�g���ɓ����
void WagnerTorusGraphsLib::DoAllVerify(bool do_chainverify) {
  int trans_count = 0;
  
  const int max_trans_count = kSqrtMaxTransCountStopper > nodelist_vector_.size() ?
                              nodelist_vector_.size() * (nodelist_vector_.size() - 1) :
                              kSqrtMaxTransCountStopper * kSqrtMaxTransCountStopper;
  // ���X�g�̖����܂Œ��ׂ�
  for (int i_diagonal = 0; i_diagonal < diagonallist_vector_.size(); i_diagonal++) {
    // �Ίp�����܂����݂��Ă��邩
    NodeQuadrangle trans_quad;
    NodePair nodepair = diagonallist_vector_[i_diagonal];
    if (!MakeQuadrangle(nodepair.nodeA, nodepair.nodeB, trans_quad)) continue;
    // diagonal transformation�̋��ʔ�����s��
    TransJudge must_trans_judge = JudgeDiagonalTransformation(trans_quad);
    if (must_trans_judge == kMustNotTrans) continue;
    if (must_trans_judge == kVerify) {
      // diagonal transformation���邩���肷��
      Vec2d trans_quad_coord[4];
      void* ainfoparam_trans_quad[4];
      for (int i = 0; i < 4; i++) {
        trans_quad_coord[i][X] = nodelist_vector_[trans_quad[i]].xy_coord_[X];
        trans_quad_coord[i][Y] = nodelist_vector_[trans_quad[i]].xy_coord_[Y];
        ainfoparam_trans_quad[i] = nodelist_vector_[trans_quad[i]].infoparam_;
      }
      bool should_trans;
      try {
        // �o�^�֐��̎g�p
        if (DiagonalVerifyFunc_ == nullptr) throw 0;
        should_trans = DiagonalVerifyFunc_(diagverifyfunc_param_, trans_quad_coord, ainfoparam_trans_quad);
      }
      catch (...) {
        // �����֐��̎g�p
        if (DiagonalVerifyFunc_ != nullptr) printf("DiagonalVerifyFunc has error.\n");
        should_trans = DefaultVerify(trans_quad_coord);
      }
      if (!should_trans) continue;
    }
    // ���s
    DoQuadDiagonalTransformation(trans_quad);
    if (do_chainverify) PushDiagonalListQuadrangle(trans_quad);
    if (++trans_count >= max_trans_count) {
      printf("DiagonalTransformation Count Over : Check DiagonalVerifyFunc is a condition for solution\n");
      break;
    }
  }
  diagonallist_vector_.clear();
}

// diagonal transformation���؂̊֐����o�^����Ă��Ȃ������ꍇ�ɁAdiagonal transformation���ׂ������肷��f�t�H���g�֐�
// �񎟌����W��ŁA�l�p�`�̑Ίp�̘a��180�x�ȏ�ɂȂ���ɑΊp���𒣂�
bool WagnerTorusGraphsLib::DefaultVerify(Vec2d coord[4]) {
  // ���݂̑Ίp��0-2�̓��ώ擾
  Vec2d vec[4];
  for (int i = 0; i < 4; i++) {
    vec[i][X] = coord[(i + 1) % 4][X] - coord[i][X];
    vec[i][Y] = coord[(i + 1) % 4][Y] - coord[i][Y];
  }
  double inner0 = -vec[0][X] * vec[3][X] - vec[0][Y] * vec[3][Y];
  double inner2 = -vec[1][X] * vec[2][X] - vec[1][Y] * vec[2][Y];
  // �ǂ������s�p�Ȃ�0-2�ɑΊp���͒���ׂ��łȂ�
  if (inner0 > 0 && inner2 > 0) return true;
  // �ǂ������݊p�Ȃ�0-2�ɑΊp���͒������܂܂��ǂ�
  if (inner0 < 0 && inner2 < 0) return false;
  // ���������ꍇ�Acos�����߂đ����ĕ��̏ꍇ�́A��p�̍��v��180�x�ȏ�ɂȂ�
  double cos0 = inner0 / sqrt((vec[0][X] * vec[0][X] + vec[0][Y] * vec[0][Y]) * (vec[3][X] * vec[3][X] + vec[3][Y] * vec[3][Y]));
  double cos2 = inner2 / sqrt((vec[2][X] * vec[2][X] + vec[2][Y] * vec[2][Y]) * (vec[1][X] * vec[1][X] + vec[1][Y] * vec[1][Y]));
  return cos0 + cos2 > 1e-20;
}

// �Y������G�b�W��Ίp���Ƃ����l�p�`�̔ԍ����擾����
// quad�̃��[����0����͂��ꂽnode�Ƃ��A�����v��菇�ɕ��ׂ�B�Ίp����0-2�Ő���
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

// ��������diagonal transformation�ł��邩
bool WagnerTorusGraphsLib::CanDiagonalTransformation(NodeQuadrangle quad) {
  // ���݂̑Ίp��0-2�̂ǂ��炩�����p
  int reentrant_vertex = QuadrantHasReentrant(quad);
  if (reentrant_vertex == 0 || reentrant_vertex == 2) return false;
  // ���݂̑Ίp��0-2���ؒf�s�\ID�ł���
  NodeInfo & info0 = nodelist_vector_[quad[0]];
  EdgeID ID = info0.GetID(info0.GetEdge(quad[2]));
  if (IsUncuttableID(ID)) return false;
  return true;
}
// diagonal transformation�����Ȃ��Ă͂Ȃ�Ȃ����A���Ă͂����Ȃ����AVerify�֐������肷��
TransJudge WagnerTorusGraphsLib::JudgeDiagonalTransformation(NodeQuadrangle quad) {
  if (!CanDiagonalTransformation(quad)) return kMustNotTrans;
  if (quad[1] < kDefaultThreeNode || quad[3] < kDefaultThreeNode) return kMustNotTrans;
  if (quad[0] < kDefaultThreeNode || quad[2] < kDefaultThreeNode) return kMustTrans;
  return kVerify;
}

// MakeQuadrangle�ō쐬���ꂽ�����Quad�ł��邱�Ƃ�O���diagonal transformation�A0-2�̐ؒf��1-3�̐ڑ����s��
bool WagnerTorusGraphsLib::DoQuadDiagonalTransformation(NodeQuadrangle quad) {
  EdgeIndex edge[4];
  // 0-2�̐ؒf
  edge[0] = nodelist_vector_[quad[0]].GetEdge(quad[2]);
  edge[2] = nodelist_vector_[quad[2]].GetEdge(quad[0]);
  if (edge[0] == kUndefined || edge[2] == kUndefined) return false;
  nodelist_vector_[quad[0]].EraseEdge(edge[0]);
  nodelist_vector_[quad[2]].EraseEdge(edge[2]);
  // 1-3�̐ڑ�
  edge[1] = nodelist_vector_[quad[1]].GetEdge(quad[2]);
  edge[3] = nodelist_vector_[quad[3]].GetEdge(quad[0]);
  if (edge[1] == kUndefined || edge[3] == kUndefined) return false;
  nodelist_vector_[quad[1]].AddEdge(quad[3], kPlane, edge[1] + 1);
  nodelist_vector_[quad[3]].AddEdge(quad[1], kPlane, edge[3] + 1);
  return true;
}

// node�̓񎟌����W����destination_coord�܂ł̃x�N�g���́A�ǂ̃G�b�W�ƃG�b�W�ɋ��܂�Ă��邩�Ԃ�
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

// coord1��coord2��coord3�ւ̌o�H�Ŕ����v���ɋȂ����Ă��邩����B���s�͒��i��true�A�t������false�Ƃ���
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

//�����v��� �����܂��͉��p���Ȃ����T�����A�������ꍇ�͂��̊p�̃C�e���[�^��Ԃ�
int WagnerTorusGraphsLib::QuadrantHasReentrant(NodeQuadrangle quad) {
  for (int iVertex = 0; iVertex < 4; iVertex++) {
    NodeTriangle tri = { quad[(iVertex + 3) % 4],quad[iVertex],quad[(iVertex + 1) % 4] };
    if (!IsLevorotation(tri) || IsStraight(tri)) return iVertex;
  }
  return kUndefined;
}

int WagnerTorusGraphsLib::FindTrianglePosition(Vec2d coord, NodeTriangle tri, NodeIndex start_node) {
  // ���s������
  auto not_found = [&] {
    tri[0] = kUndefined;
    tri[1] = kUndefined;
    tri[2] = kUndefined;
  };
  // �ő�l�_��ۑ�����T�����s��
  NodeQuadrangle circling_cuad = { start_node, kUndefined, kUndefined, kUndefined };
  NodeIndex find_count = 0;
  const NodeIndex max_find_count = NodeIndex(nodelist_vector_.size());
  char vertex = 0;
  while (find_count++ < max_find_count) {
    // ���݂�vertex����T�����W�Ɍ����ڑ��𒲂ׂ�
    EdgeIndex approaches_edge = GetSandwichVectorEdge(circling_cuad[vertex], coord);
    NodeIndex next_node = GetAccessNode(circling_cuad[vertex], approaches_edge);
    // �o�^����vertex�������ӏ�������Ă��Ȃ����ᖡ
    {
      if (IsSameCoord(coord, nodelist_vector_[next_node].xy_coord_)) {
        tri[0] = next_node;
        tri[1] = next_node;
        tri[2] = next_node;
        return 1;
      }
      // 3�O�Ɠ����_�Ɍ������Ă�����A�l�p�`������Ă���B�Ίp�������B
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
      // 2�O�Ɠ����_�Ɍ������Ă�����O�p�`������Ă���B�O�p�`�����B
      if (next_node == circling_cuad[(vertex + 2) % 4]) {
        tri[0] = circling_cuad[(vertex + 2) % 4];
        tri[1] = circling_cuad[(vertex + 3) % 4];
        tri[2] = circling_cuad[(vertex + 4) % 4];
        return 3;
      }
      // 1�O�Ɠ����_�Ɍ������Ă�����A��ӂ�����Ă���B���������B
      if (next_node == circling_cuad[(vertex + 3) % 4]) {
        tri[0] = circling_cuad[(vertex + 4) % 4];
        tri[1] = circling_cuad[(vertex + 3) % 4];
        tri[2] = circling_cuad[(vertex + 3) % 4];
        return 2;
      }
      // ��O�Ɠ������Ƃ͒ʏ킠�蓾�Ȃ�
      if (next_node == circling_cuad[(vertex + 4) % 4]) {
        not_found();
        return 0;
      }
    }
    // ������Ȃ������ꍇ�́Acircling_cuad�̒l���X�V���Ď��̓_�𒲂ׂ�
    vertex = (vertex + 1) % 4;
    circling_cuad[vertex] = next_node;
  }
  not_found();
  return 0;
}

// �w�肳�ꂽ�O�p�`��œ_�̓o�^���m�肵�A�m�肵���_�ԍ���Ԃ��B�K�v�Ȃ�diagonal transformation������s���B
NodeIndex WagnerTorusGraphsLib::DetermineNodeOnTriangle(NodeInfo& node, NodeTriangle tri, bool do_chainverify) {
  EdgeID ID = GetTriangleEdgeID(tri);
  NodeIndex num_newnode = nodelist_vector_.size();
  for (int i = 0; i < 3; i++) {
    // �V�_����O�p�`���֐ڑ�
    node.accessnodelist_vector_.push_back(tri[i]);
    node.accessidlist_vector_.push_back(InvertedID(ID));
    // �O�p�`������V�_�֐ڑ�
    NodeInfo& info = nodelist_vector_[tri[i]];
    EdgeIndex edge = info.GetEdge(tri[(i + 1) % 3]);
    info.AddEdge(num_newnode, ID, edge+1);
  }
  // �V�_�o�^
  nodelist_vector_.push_back(node);
  // �O�p�`����̐���
  if (do_chainverify) {
    PushDiagonalListTriangle(tri);
    DoAllVerify();
  }
  return num_newnode;
}

// �w�肳�ꂽ�ӏ�œ_�̓o�^���m�肵�A�m�肵���_�ԍ���Ԃ��B�K�v�Ȃ�diagonal transformation������s���B
NodeIndex WagnerTorusGraphsLib::DetermineNodeOnEdge(NodeInfo& node, NodePair line, bool do_chainverify) {
  NodeQuadrangle quad;
  MakeQuadrangle(line.nodeA, line.nodeB, quad);
  EdgeID ID = GetTriangleEdgeID(quad);
  NodeIndex num_newnode = nodelist_vector_.size();
  for (int i = 0; i < 4; i++) {
    // �V�_����l�p�`���֐ڑ�
    node.accessnodelist_vector_.push_back(quad[i]);
    node.accessidlist_vector_.push_back(InvertedID(ID));
    // �l�p�`������V�_�֐ڑ�
    if (i == 0 || i == 2) {
      // �Ίp�����͐ؒf���ď���ς���
      NodeInfo& info = nodelist_vector_[quad[i]];
      EdgeIndex edge = nodelist_vector_[quad[i]].GetEdge(quad[(i + 2) % 4]);
      info.accessnodelist_vector_[edge] = num_newnode;
      info.accessidlist_vector_[edge] = ID;
    }
    else {
      // �Ίp���łȂ��ق��͒ǉ�
      NodeInfo& info = nodelist_vector_[quad[i]];
      EdgeIndex edge = info.GetEdge(quad[(i + 1) % 4]);
      info.AddEdge(num_newnode, ID, edge+1);
    }
  }
  // �V�_�o�^
  nodelist_vector_.push_back(node);
  // �O�p�`����̐���
  if (do_chainverify) {
    PushDiagonalListQuadrangle(quad);
    DoAllVerify();
  }
  return num_newnode;
}

// ��_��؂�
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

// kSameLocationDist�ɏ]������_�����肷��
bool WagnerTorusGraphsLib::IsSameCoord(Vec2d coord1, Vec2d coord2) {
  return abs(coord1[X] - coord2[X]) < kSameLocationDist && abs(coord1[Y] - coord2[Y]) < kSameLocationDist;
}
bool WagnerTorusGraphsLib::IsSameNode(NodeIndex node1, NodeIndex node2) {
  return IsSameCoord(nodelist_vector_[node1].xy_coord_, nodelist_vector_[node2].xy_coord_);
}

// ID���؂���邩�ۂ��ikUncuttableIDList���Q�Ɓj
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

// �O�p�`�ɒǉ�����G�b�W�̑�����Ԃ��B�����g���p
EdgeID WagnerTorusGraphsLib::GetTriangleEdgeID(NodeTriangle tri) {
  return kPlane;
}

} //namespace

