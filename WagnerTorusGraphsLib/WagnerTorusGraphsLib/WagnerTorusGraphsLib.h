#pragma once
#ifndef WAGNERTORUSGRAPHSLIB_WAGNERTORUSGRAPHSLIB_H_
#define WAGNERTORUSGRAPHSLIB_WAGNERTORUSGRAPHSLIB_H_

#include <vector>

namespace WagnerTorusGraphs {

using NodeIndex = int;  //�O���t�̃m�[�h�i�_�j��\���C�e���[�^
using EdgeIndex = short;  //�m�[�h���̃G�b�W�i�Ӂj��\���C�e���[�^
using Vec2d = double[2];
enum { X, Y };

struct NodePair {NodeIndex nodeA; NodeIndex nodeB;};
using NodeTriangle = NodeIndex[3];
using NodeQuadrangle = NodeIndex[4];

// �e��(�֐��ɑΉ������p�����[�^�|�C���^, xy_coord_(�_�ʓ񎟌����), infoparam_(�_�ʌʏ��))��������
using DiagonalVerifyFunc = bool(*)(void*, Vec2d[4], void* [4]);
using GetNodeFunc = void(*)(void*, Vec2d, void*);
using GetEdgeFunc = void(*)(void*, Vec2d[2], void* [2]);
using GetTriangleFunc = void(*)(void*, Vec2d[3], void* [3]);

const int kUndefined = -1;
enum EdgeID { kPlane, kDefault, kUncuttable };  // �G�b�W�̐ڑ����B���T���v���ł́A�f�t�H���g�Ɛؒf�s�\�̂�
const EdgeID kUncuttableIDList[] = { kDefault, kUncuttable }; // �ڑ������������Ȃ�ID
const EdgeID kInvertIDList[][2] = { {kDefault,kDefault} };  // ��_�őΏƓI�ɗ^������ID(2019/04/17 : ���݂͑΂Ƃ��Ă��鑮���͂Ȃ����߉��̎҂̂ݓ���Ă���j
enum TransJudge { kVerify, kMustTrans, kMustNotTrans };

const int kDefaultThreeNode = 3; //�Ȃɂ��_��o�^���Ȃ��Ă��A�\�񂳂ꂽ�_���O�_���݂��邱�Ƃ��R�[�h��Ő�������
const double kSameLocationDist = 1e-8;  // X����Y�������ɂ��̒l������XY�����͓���_�Ƃ݂Ȃ�
const double kCoordLimit = 100;  // 2�������ʂ̎�肤����W��Βl�̍ő�i�傫������ƌv�Z�Ɏ��s����j
const int kSqrtMaxTransCountStopper = 30000;  // �Ίp�ό`���s���񐔂̏���̕������Bint�̏���𒴂��Ȃ��悤�ɂ���

// �m�[�h���B�f�[�^�^���p�\���̂Ƃ��Ċ�{�I��public�ň���
class NodeInfo {
  friend class WagnerTorusGraphsLib;
protected:
  std::vector<NodeIndex> accessnodelist_vector_;  // IDList�Ɠ����B�����v��菇�ɓo�^����
  std::vector<EdgeID> accessidlist_vector_;  // NodeList�Ɠ����C�e���[�^�̐ڑ����B�����v��菇�ɓo�^����
public:
  Vec2d xy_coord_; // �m�[�h�̓񎟌����i�K�{�j
  void* infoparam_; // �m�[�h�̌ʏ��|�C���^�i�C�Ӂj

  inline void SetCoord(double x_coord, double y_coord) {
    xy_coord_[X] = x_coord;
    xy_coord_[Y] = y_coord;
  }
  inline EdgeIndex GetEdge(NodeIndex node) {
    for (EdgeIndex i = 0; i < accessnodelist_vector_.size(); i++) {
      if (accessnodelist_vector_[i] == node) return i;
    }
    return kUndefined;
  }
  inline EdgeIndex GetAccessSize() {
    return EdgeIndex(accessnodelist_vector_.size());
  }
  inline NodeIndex GetNode(EdgeIndex edge) {
    return accessnodelist_vector_[(edge + accessnodelist_vector_.size()) % accessnodelist_vector_.size()];
  }
  inline EdgeID GetID(EdgeIndex edge) {
    return accessidlist_vector_[(edge + accessnodelist_vector_.size()) % accessnodelist_vector_.size()];
  }
  // �Y������EdgeLine�C�e���[�^�̒���ɁA�G�b�W��ǉ�
  void AddEdge(NodeIndex node, EdgeID ID, EdgeIndex edge) {
    accessnodelist_vector_.insert(accessnodelist_vector_.begin() + edge, node);
    accessidlist_vector_.insert(accessidlist_vector_.begin() + edge, ID);
  }
  // �Y������EdgeLine�C�e���[�^���폜
  void EraseEdge(EdgeIndex edge) {
    accessnodelist_vector_.erase(accessnodelist_vector_.begin() + edge);
    accessidlist_vector_.erase(accessidlist_vector_.begin() + edge);
  }
  // �Y������NodeIndex������ꍇ�폜
  inline void EraseEdge(NodeIndex node) {
    EdgeIndex edge = GetEdge(node);
    if (edge != kUndefined) EraseEdge(edge);
  }
  NodeInfo(double x_coord = 0, double y_coord = 0) {
    xy_coord_[X] = x_coord;
    xy_coord_[Y] = y_coord;
    infoparam_ = nullptr;
    accessnodelist_vector_.reserve(8);
    accessidlist_vector_.reserve(8);
  }
};

class WagnerTorusGraphsLib {
public:
  WagnerTorusGraphsLib();
  ~WagnerTorusGraphsLib();
  inline void SetDiagonalVerifyFunc(void* param, DiagonalVerifyFunc func) { diagverifyfunc_param_ = param; DiagonalVerifyFunc_ = func; }

  NodeIndex RegistNode(NodeInfo& node);  // �_�̓o�^���s���A�o�^���ꂽ�_�̃m�[�h�ԍ���Ԃ�
  bool DeleteNode(NodeIndex node, bool do_verify = true);  // �_�̍폜

  int BridgeNode(NodeIndex node1, NodeIndex node2, bool do_verify = true, int regression_count = 0/*�K��0����J�n*/); //��_�ڑ�
  bool SetEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID); // �ڑ�����Ă����_��ID�o�^
  bool SetAllEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID); // �ڑ�����Ă����_��ID�o�^
  bool DiagonalTransformation(NodeIndex node, EdgeIndex edge); // �ڑ�����Ă����_��diagonal transformation���s��
  bool DiagonalTransformation(NodeIndex node1, NodeIndex node2);
  void AllVerifyDiagonalTransformation();  // �S�Ă̐ڑ��ɑ΂��āAdiagonal transformation���s����DiagonalVerifyFunc�Ō��؂���diagonal transformation���s���B

  NodeInfo GetNodeInfo(NodeIndex node) { return nodelist_vector_[node]; };
  void GetNode(void* param, GetNodeFunc func);  // �����_�ƍ폜�_���������S�Ă̓_��func��ʂ�
  void GetEdge(void* param, GetEdgeFunc func);  // �S�Ă̐ڑ��ɑ΂���func��ʂ�
  void GetTriangle(void* param, GetTriangleFunc func);  // �S�Ă̎O�p�`�ɑ΂���func��ʂ�
  
  EdgeID InvertedID(EdgeID ID); // ���͂���ID�̑΂ɂȂ�ID���o�́B�g���p
  inline NodeIndex GetLastNode() {
    NodeIndex last_node = NodeIndex(nodelist_vector_.size()) - 1;
    while (IsDeleted(last_node)) {
      --last_node;
      if (last_node < kDefaultThreeNode) return kUndefined;
    }
    return last_node;
  }

protected:
  //do_chainverify : diagonal transformation��O�p�`�o�^�ȂǁA�������瑀����s�����O�p�`��l�p�`�ɑ΂��āA���̕ӂ����ؑΏۂɂ���B���؂���diagonal transformation������X�ɂ�������ؑΏۂƂ���
  void AddAllEdgeToDiagonalList();  // �S�Ă̐ڑ���diagonallist�ɓ����
  void DoAllVerify(bool do_chainverify = true);  // diagonallist�̑S�Ă����؋y��diagonal transformation���s��
  bool DefaultVerify(Vec2d coord[4]); // diagonal transformation����֐�DiagonalVerifyFunc_������`�������͎Q�Ǝ��s�����ꍇ�Ɏg�p�����f�t�H���g�֐�(true : diagonal transformation�s���ׂ�)
  bool MakeQuadrangle(NodeIndex node, EdgeIndex edge, NodeQuadrangle quad); // �w��G�b�W����l�p�`�_����쐬(false : �쐬���s)
  bool MakeQuadrangle(NodeIndex node1, NodeIndex node2, NodeQuadrangle quad);
  bool CanDiagonalTransformation(NodeQuadrangle quad); // �w��l�p�`��diagonal transformation�\�ł���Œ�����𖞂����Ă��邩
  TransJudge JudgeDiagonalTransformation(NodeQuadrangle quad); // �w��l�p�`��diagonal transformation�����ق����ǂ������������Ă��邩�R�����g����
  bool DoQuadDiagonalTransformation(NodeQuadrangle quad);  // �w��l�p�`��diagonal transformation�����s����

  EdgeIndex GetSandwichVectorEdge(NodeInfo& node, Vec2d destination_coord);  // node�̍��W��destination_coord�����񂾍ۂɁA�ǂ̃G�b�W�Ƃǂ̃G�b�W�ɋ��܂�Ă��邩�Ԃ�(�߂�l�Ɩ߂�l+1�ɋ��܂�Ă���j
  EdgeIndex GetSandwichVectorEdge(NodeIndex node, Vec2d destination_coord);
  bool IsLevorotation(Vec2d coord1, Vec2d coord2, Vec2d coord3); // �O�_�����Ԃƍ����ɓ����Ă��邩���肷��
  bool IsLevorotation(NodeTriangle Tri);
  bool IsStraight(Vec2d coord1, Vec2d coord2, Vec2d coord3); // �O�_�����Ԃƍ����ɓ����Ă��邩���肷��
  bool IsStraight(NodeTriangle Tri);

  int QuadrantHasReentrant(NodeQuadrangle quad);  // ���p���������l�p�`�����肷��

  int FindTrianglePosition(Vec2d coord, NodeTriangle tri, NodeIndex start_node); // start_node����T���J�n���āAcoord������Ɏ��O�p�`�܂��͏悹�Ă���G�b�W��tri�Ɋi�[����(�G�b�W�̏ꍇ2,�O�p�`�̏ꍇ3��Ԃ�)
  NodeIndex DetermineNodeOnTriangle(NodeInfo& node, NodeTriangle tri, bool do_chainverify = true); // �w�肳�ꂽ�O�p�`tri��ɐV����node��o�^������node�ԍ���Ԃ�
  NodeIndex DetermineNodeOnEdge(NodeInfo& node, NodePair line, bool do_chainverify = true); // �w�肳�ꂽ��line��ɐV����node��o�^������node�ԍ���Ԃ�

  bool CutEdge(NodeIndex node1, NodeIndex node2); // ��_�Ԃ̃G�b�W�ڑ��𖕏�����

  bool IsSameCoord(Vec2d coord1, Vec2d coord2); // �����_�Ɣ���
  bool IsSameNode(NodeIndex node1, NodeIndex node2);  // �����_�̃m�[�h�Ɣ���
  bool IsUncuttableID(EdgeID ID); // diagonal transformation��؂��肪�\�ȃG�b�W�ł���
  bool HasUncuttableID(NodeIndex node); // �؂�Ȃ��G�b�W���������m�[�h�ԍ��ł���
  EdgeID GetTriangleEdgeID(NodeTriangle tri);  // �O�p�`��ID���v�Z����i�g���p�j

  inline void GetCoord(NodeIndex node, Vec2d coord) {
    coord[0] = nodelist_vector_[node].xy_coord_[0];
    coord[1] = nodelist_vector_[node].xy_coord_[1];
  }
  inline void PushDiagonalListTriangle(NodeTriangle tri) {
    for (int i = 0; i < 3; i++) {
      NodePair pair{ tri[i], tri[(i + 1) % 3] };
      diagonallist_vector_.push_back(pair);
    }
  }
  inline void PushDiagonalListQuadrangle(NodeQuadrangle quad) {
    for (int i = 0; i < 4; i++) {
      NodePair pair{ quad[i], quad[(i + 1) % 4] };
      diagonallist_vector_.push_back(pair);
    }
  }
  inline bool IsDeleted(NodeIndex node) {
    return nodelist_vector_[node].accessidlist_vector_.empty();
  }
  inline NodeIndex GetAccessNode(NodeIndex node, EdgeIndex edge) {
    return nodelist_vector_[node].accessnodelist_vector_[edge];
  }
  inline EdgeID GetAccessID(NodeIndex node, EdgeIndex edge) {
    return nodelist_vector_[node].accessidlist_vector_[edge];
  }


private:
  std::vector<NodeInfo> nodelist_vector_; // �S�m�[�h�f�[�^���X�g
  std::vector<NodePair> diagonallist_vector_;  // diagonal transformation���̑Ίp�����X�g
  DiagonalVerifyFunc DiagonalVerifyFunc_; // diagonal transformation���s�������肷��֐�
  void* diagverifyfunc_param_;  // ��L�֐���ǂ񂾍ۂɌĂэ��߂�|�C���^
};
#endif //WAGNERTORUSGRAPHSLIB_WAGNERTORUSGRAPHSLIB_H_

} //namespase