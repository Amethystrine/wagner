#pragma once
#ifndef WAGNERTORUSGRAPHSLIB_WAGNERTORUSGRAPHSLIB_H_
#define WAGNERTORUSGRAPHSLIB_WAGNERTORUSGRAPHSLIB_H_

#include <vector>

namespace WagnerTorusGraphs {

using NodeIndex = int;  //グラフのノード（点）を表すイテレータ
using EdgeIndex = short;  //ノード毎のエッジ（辺）を表すイテレータ
using Vec2d = double[2];
enum { X, Y };

struct NodePair {NodeIndex nodeA; NodeIndex nodeB;};
using NodeTriangle = NodeIndex[3];
using NodeQuadrangle = NodeIndex[4];

// 各種(関数に対応したパラメータポインタ, xy_coord_(点別二次元情報), infoparam_(点別個別情報))が送られる
using DiagonalVerifyFunc = bool(*)(void*, Vec2d[4], void* [4]);
using GetNodeFunc = void(*)(void*, Vec2d, void*);
using GetEdgeFunc = void(*)(void*, Vec2d[2], void* [2]);
using GetTriangleFunc = void(*)(void*, Vec2d[3], void* [3]);

const int kUndefined = -1;
enum EdgeID { kPlane, kDefault, kUncuttable };  // エッジの接続情報。当サンプルでは、デフォルトと切断不可能のみ
const EdgeID kUncuttableIDList[] = { kDefault, kUncuttable }; // 接続解除をさせないID
const EdgeID kInvertIDList[][2] = { {kDefault,kDefault} };  // 二点で対照的に与えたいID(2019/04/17 : 現在は対としている属性はないため仮の者のみ入れている）
enum TransJudge { kVerify, kMustTrans, kMustNotTrans };

const int kDefaultThreeNode = 3; //なにも点を登録しなくても、予約された点が三点存在することをコード上で説明する
const double kSameLocationDist = 1e-8;  // X方向Y方向共にこの値未満のXY距離は同一点とみなす
const double kCoordLimit = 100;  // 2次元平面の取りうる座標絶対値の最大（大きすぎると計算に失敗する）
const int kSqrtMaxTransCountStopper = 30000;  // 対角変形を行う回数の上限の平方根。intの上限を超えないようにする

// ノード情報。データ運搬用構造体として基本的にpublicで扱う
class NodeInfo {
  friend class WagnerTorusGraphsLib;
protected:
  std::vector<NodeIndex> accessnodelist_vector_;  // IDListと同期。反時計回り順に登録する
  std::vector<EdgeID> accessidlist_vector_;  // NodeListと同じイテレータの接続情報。反時計回り順に登録する
public:
  Vec2d xy_coord_; // ノードの二次元情報（必須）
  void* infoparam_; // ノードの個別情報ポインタ（任意）

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
  // 該当するEdgeLineイテレータの直後に、エッジを追加
  void AddEdge(NodeIndex node, EdgeID ID, EdgeIndex edge) {
    accessnodelist_vector_.insert(accessnodelist_vector_.begin() + edge, node);
    accessidlist_vector_.insert(accessidlist_vector_.begin() + edge, ID);
  }
  // 該当するEdgeLineイテレータを削除
  void EraseEdge(EdgeIndex edge) {
    accessnodelist_vector_.erase(accessnodelist_vector_.begin() + edge);
    accessidlist_vector_.erase(accessidlist_vector_.begin() + edge);
  }
  // 該当するNodeIndexがある場合削除
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

  NodeIndex RegistNode(NodeInfo& node);  // 点の登録を行い、登録された点のノード番号を返す
  bool DeleteNode(NodeIndex node, bool do_verify = true);  // 点の削除

  int BridgeNode(NodeIndex node1, NodeIndex node2, bool do_verify = true, int regression_count = 0/*必ず0から開始*/); //二点接続
  bool SetEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID); // 接続されている二点にID登録
  bool SetAllEdgeID(NodeIndex node1, NodeIndex node2, EdgeID ID); // 接続されている二点にID登録
  bool DiagonalTransformation(NodeIndex node, EdgeIndex edge); // 接続されている二点のdiagonal transformationを行う
  bool DiagonalTransformation(NodeIndex node1, NodeIndex node2);
  void AllVerifyDiagonalTransformation();  // 全ての接続に対して、diagonal transformationを行うかDiagonalVerifyFuncで検証してdiagonal transformationを行う。

  NodeInfo GetNodeInfo(NodeIndex node) { return nodelist_vector_[node]; };
  void GetNode(void* param, GetNodeFunc func);  // 初期点と削除点を除いた全ての点でfuncを通す
  void GetEdge(void* param, GetEdgeFunc func);  // 全ての接続に対してfuncを通す
  void GetTriangle(void* param, GetTriangleFunc func);  // 全ての三角形に対してfuncを通す
  
  EdgeID InvertedID(EdgeID ID); // 入力したIDの対になるIDを出力。拡張用
  inline NodeIndex GetLastNode() {
    NodeIndex last_node = NodeIndex(nodelist_vector_.size()) - 1;
    while (IsDeleted(last_node)) {
      --last_node;
      if (last_node < kDefaultThreeNode) return kUndefined;
    }
    return last_node;
  }

protected:
  //do_chainverify : diagonal transformationや三角形登録など、何かしら操作を行った三角形や四角形に対して、その辺も検証対象にする。検証してdiagonal transformationしたら更にそれも検証対象とする
  void AddAllEdgeToDiagonalList();  // 全ての接続をdiagonallistに入れる
  void DoAllVerify(bool do_chainverify = true);  // diagonallistの全てを検証及びdiagonal transformationを行う
  bool DefaultVerify(Vec2d coord[4]); // diagonal transformation判定関数DiagonalVerifyFunc_が未定義もしくは参照失敗した場合に使用されるデフォルト関数(true : diagonal transformation行うべき)
  bool MakeQuadrangle(NodeIndex node, EdgeIndex edge, NodeQuadrangle quad); // 指定エッジから四角形点列を作成(false : 作成失敗)
  bool MakeQuadrangle(NodeIndex node1, NodeIndex node2, NodeQuadrangle quad);
  bool CanDiagonalTransformation(NodeQuadrangle quad); // 指定四角形がdiagonal transformation可能である最低条件を満たしているか
  TransJudge JudgeDiagonalTransformation(NodeQuadrangle quad); // 指定四角形がdiagonal transformationしたほうが良い条件を持っているかコメントする
  bool DoQuadDiagonalTransformation(NodeQuadrangle quad);  // 指定四角形のdiagonal transformationを実行する

  EdgeIndex GetSandwichVectorEdge(NodeInfo& node, Vec2d destination_coord);  // nodeの座標とdestination_coordを結んだ際に、どのエッジとどのエッジに挟まれているか返す(戻り値と戻り値+1に挟まれている）
  EdgeIndex GetSandwichVectorEdge(NodeIndex node, Vec2d destination_coord);
  bool IsLevorotation(Vec2d coord1, Vec2d coord2, Vec2d coord3); // 三点を結ぶと左回りに動いているか判定する
  bool IsLevorotation(NodeTriangle Tri);
  bool IsStraight(Vec2d coord1, Vec2d coord2, Vec2d coord3); // 三点を結ぶと左回りに動いているか判定する
  bool IsStraight(NodeTriangle Tri);

  int QuadrantHasReentrant(NodeQuadrangle quad);  // 凹角を持った四角形か判定する

  int FindTrianglePosition(Vec2d coord, NodeTriangle tri, NodeIndex start_node); // start_nodeから探索開始して、coordを内側に持つ三角形または乗せているエッジをtriに格納する(エッジの場合2,三角形の場合3を返す)
  NodeIndex DetermineNodeOnTriangle(NodeInfo& node, NodeTriangle tri, bool do_chainverify = true); // 指定された三角形tri上に新しくnodeを登録しそのnode番号を返す
  NodeIndex DetermineNodeOnEdge(NodeInfo& node, NodePair line, bool do_chainverify = true); // 指定された辺line上に新しくnodeを登録しそのnode番号を返す

  bool CutEdge(NodeIndex node1, NodeIndex node2); // 二点間のエッジ接続を抹消する

  bool IsSameCoord(Vec2d coord1, Vec2d coord2); // 同じ点と判定
  bool IsSameNode(NodeIndex node1, NodeIndex node2);  // 同じ点のノードと判定
  bool IsUncuttableID(EdgeID ID); // diagonal transformationや切り取りが可能なエッジである
  bool HasUncuttableID(NodeIndex node); // 切れないエッジを持ったノード番号である
  EdgeID GetTriangleEdgeID(NodeTriangle tri);  // 三角形のIDを計算する（拡張用）

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
  std::vector<NodeInfo> nodelist_vector_; // 全ノードデータリスト
  std::vector<NodePair> diagonallist_vector_;  // diagonal transformation候補の対角線リスト
  DiagonalVerifyFunc DiagonalVerifyFunc_; // diagonal transformationを行うか判定する関数
  void* diagverifyfunc_param_;  // 上記関数を読んだ際に呼び込めるポインタ
};
#endif //WAGNERTORUSGRAPHSLIB_WAGNERTORUSGRAPHSLIB_H_

} //namespase