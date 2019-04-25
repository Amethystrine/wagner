#include <stdio.h>
#include "WagnerTorusGraphsLib.h"
#include <time.h>


void AddTriangleArea(void* funcparam, double coord[3][2], void* nodeparam[3], WagnerTorusGraphs::EdgeID ID[3]) {
  double* area_sum = static_cast<double*>(funcparam);
  // �O�p�`�͍��W��͕K�������v��肪��������]�Ƃ��Ĉ����Ă���
  *area_sum += (coord[0][0] * coord[1][1] + coord[1][0] * coord[2][1] + coord[2][0] * coord[0][1] 
              - coord[0][1] * coord[1][0] - coord[1][1] * coord[2][0] - coord[2][1] * coord[0][0]) / 2;
}

void SetArrow(void* funcparam, double coord[2][2], void* nodeparam[2], WagnerTorusGraphs::EdgeID ID) {
  FILE* file = static_cast<FILE*>(funcparam);
  int* node0 = static_cast<int*>(nodeparam[0]);
  int* node1 = static_cast<int*>(nodeparam[1]);
  if (ID == WagnerTorusGraphs::kDirectedGraph_Initial) {
    fprintf(file, " set arrow %d from %f,%f to %f,%f filled linewidth 2.0\n", *node0, coord[0][0], coord[0][1], coord[1][0], coord[1][1]);
  }
  if (ID == WagnerTorusGraphs::kDirectedGraph_Terminal) {
    fprintf(file, " set arrow %d from %f,%f to %f,%f filled linewidth 2.0\n", *node1, coord[1][0], coord[1][1], coord[0][0], coord[0][1]);
  }
}

// plot "�t�@�C����" with linespoints��gnuplot�ɓ��͂��Ă݂悤
void OutputForGnuplotWithLinesPoints(void* funcparam, double coord[2][2], void* nodeparam[2], WagnerTorusGraphs::EdgeID ID) {
  FILE* file = static_cast<FILE*>(funcparam);
  fprintf(file, "%f %f\n", coord[0][0], coord[0][1]);
  fprintf(file, "%f %f\n\n", coord[1][0], coord[1][1]);
}

// DefaultVerify��Z���W����������ŁA����������ɉ�����
bool test3VerifyFunc(void* funcparam, double coord[4][2], void* nodeparam[4]) {
  double* hogedouble = static_cast<double*>(funcparam); // ����͎g��Ȃ�
  double coord_z[4];
  for (int i = 0; i < 4; i++) {
    coord_z[i] = *static_cast<double*>(nodeparam[i]);
  }
  // �l�p�`0123�ɑ΂��āA0��2��Ίp���Ō��Ԃׂ��Ȃ�false,1��3�Ō��΂��ׂ��Ȃ�true��Ԃ�
  double vec[4][3];
  for (int i = 0; i < 4; i++) {
    vec[i][0] = coord[(i + 1) % 4][0] - coord[i][0];
    vec[i][1] = coord[(i + 1) % 4][1] - coord[i][1];
    vec[i][2] = coord_z[(i + 1) % 4] - coord_z[i];
  }
  double inner0 = -vec[0][0] * vec[3][0] - vec[0][1] * vec[3][1] - vec[0][2] * vec[3][2];
  double inner2 = -vec[1][0] * vec[2][0] - vec[1][1] * vec[2][1] - vec[1][2] * vec[2][2];
  // �ǂ������s�p�Ȃ�0-2�ɑΊp���͒���ׂ��łȂ�
  if (inner0 > 0 && inner2 > 0) return true;
  // �ǂ������݊p�Ȃ�0-2�ɑΊp���͒������܂܂��ǂ�
  if (inner0 < 0 && inner2 < 0) return false;
  // ���������ꍇ�Acos�����߂đ����ĕ��̏ꍇ�́A��p�̍��v��180�x�ȏ�ɂȂ�
  double cos0 = inner0 / sqrt((vec[0][0] * vec[0][0] + vec[0][1] * vec[0][1] + vec[0][2] * vec[0][2]) * (vec[3][0] * vec[3][0] + vec[3][1] * vec[3][1] + vec[3][2] * vec[3][2]));
  double cos2 = inner2 / sqrt((vec[2][0] * vec[2][0] + vec[2][1] * vec[2][1] + vec[2][2] * vec[2][2]) * (vec[1][0] * vec[1][0] + vec[1][1] * vec[1][1] + vec[1][2] * vec[1][2]));
  return cos0 + cos2 > 1e-20;

}

// �L���O���t������
int test5() {
  WagnerTorusGraphs::WagnerTorusGraphsLib wagner;
  int nodenum[1002];
  for (int i = 0; i < 1000; i++) {
    WagnerTorusGraphs::NodeInfo info(double(rand()) / RAND_MAX, double(rand()) / RAND_MAX);
    nodenum[i] = i;
    info.infoparam_ = &nodenum[i];
    wagner.RegistNode(info);
  }
  WagnerTorusGraphs::NodeInfo startnode(0, 0);
  WagnerTorusGraphs::NodeInfo goalnode(1, 1);
  int nodenum_start = 1000;
  int nodenum_goal = 1001;
  startnode.infoparam_ = &nodenum_start;
  goalnode.infoparam_ = &nodenum_goal;
  int start = wagner.RegistNode(startnode);
  int goal = wagner.RegistNode(goalnode);  
  // �ڑ��͂����L���O���tID�t�^
  wagner.SetAllEdgeID(start, goal, WagnerTorusGraphs::kDirectedGraph_Initial);
  FILE* file = nullptr;
  fopen_s(&file, "test5_1000.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);
  fopen_s(&file, "test5_command.txt", "w");
  wagner.GetEdge(file, SetArrow);
  fprintf(file, "plot \"test5_1000.txt\" with linespoints\n");
  return 0;
}



// �}�`�̓ʕ�ʐς����߂Ă݂�
int test4() {
  WagnerTorusGraphs::WagnerTorusGraphsLib wagner;
  for (int i = 0; i < 10; i++) {
    WagnerTorusGraphs::NodeInfo info(double(rand()) / RAND_MAX, double(rand()) / RAND_MAX);
    wagner.RegistNode(info);
  }
  double area_sum = 0;
  wagner.GetTriangle(&area_sum, AddTriangleArea);
  printf("\narea10:%f\n", area_sum);
  for (int i = 10; i < 100; i++) {
    WagnerTorusGraphs::NodeInfo info(double(rand()) / RAND_MAX, double(rand()) / RAND_MAX);
    wagner.RegistNode(info);
  }
  area_sum = 0;
  wagner.GetTriangle(&area_sum, AddTriangleArea);
  printf("area100:%f\n", area_sum);
  return 0;
}

// �֐��o�^�����A�ϑ��I�ȎO�p�`���������
int test3() {
  WagnerTorusGraphs::WagnerTorusGraphsLib wagner;
  double z_coord[1000];
  for (int i = 0; i < 1000; i++) {
    WagnerTorusGraphs::NodeInfo info(double(rand()) / RAND_MAX, double(rand()) / RAND_MAX);
    z_coord[i] = info.xy_coord_[0] + info.xy_coord_[1] * 10;    // y�������������d��z���W��������
    info.infoparam_ = &z_coord[i];
    wagner.RegistNode(info);
  }
  // z���W���l���Ȃ�
  FILE* file = nullptr;
  fopen_s(&file, "test3_xy.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);
  // z���W���l����
  double hogedouble = 3.3;
  wagner.SetDiagonalVerifyFunc(&hogedouble, test3VerifyFunc);
  wagner.AllVerifyDiagonalTransformation();  
  fopen_s(&file, "test3_xyz.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);
  
  return 0;
}

// �Ƃɂ�����������o�^���āA���n��������
int test2() {
  WagnerTorusGraphs::WagnerTorusGraphsLib wagner;
  for (int i = 0; i < 100000; i++) {
    WagnerTorusGraphs::NodeInfo info(double(rand())/RAND_MAX, double(rand()) / RAND_MAX);
    wagner.RegistNode(info);
  }
  if (wagner.BridgeNode(12, 7) != -1) {
    wagner.SetAllEdgeID(12, 7, WagnerTorusGraphs::kUncuttable);
  }
  FILE* file = nullptr;
  fopen_s(&file, "test2_100000.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);
  return 0;
}

// �_��o�^���āA�q���ŁA����
int test1() {
  WagnerTorusGraphs::WagnerTorusGraphsLib wagner;
  WagnerTorusGraphs::NodeInfo info0(-4, 1);
  WagnerTorusGraphs::NodeInfo info1(-4, 0);
  WagnerTorusGraphs::NodeInfo info2(-4, -1);
  WagnerTorusGraphs::NodeInfo info3(-2, 1);
  WagnerTorusGraphs::NodeInfo info4(-2, 0);
  WagnerTorusGraphs::NodeInfo info5(-2, -1);
  WagnerTorusGraphs::NodeInfo info6(0, 1);
  WagnerTorusGraphs::NodeInfo info7(0, -1);
  WagnerTorusGraphs::NodeInfo info8(2, 1);
  WagnerTorusGraphs::NodeInfo info9(2, 0);
  WagnerTorusGraphs::NodeInfo info10(2, -1);
  WagnerTorusGraphs::NodeInfo info11(4, 1);
  WagnerTorusGraphs::NodeInfo info12(4, 0);
  WagnerTorusGraphs::NodeInfo info13(4, -1);

  wagner.RegistNode(info0);
  int node1 = wagner.RegistNode(info1);
  wagner.RegistNode(info2);
  wagner.RegistNode(info3);
  wagner.RegistNode(info4);
  wagner.RegistNode(info5);
  wagner.RegistNode(info6);
  wagner.RegistNode(info7);
  wagner.RegistNode(info8);
  wagner.RegistNode(info9);
  wagner.RegistNode(info10);
  wagner.RegistNode(info11);
  int node2 = wagner.RegistNode(info12);
  int node3 = wagner.RegistNode(info13);
  FILE* file = nullptr;
  fopen_s(&file, "test1_Regist.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);

  if (wagner.BridgeNode(node1, node2) != -1) {
    wagner.SetAllEdgeID(node1, node2, WagnerTorusGraphs::kUncuttable);
    wagner.AllVerifyDiagonalTransformation(); // �ڑ������牘���Ȃ�ꍇ������̂ł�������L���C�ɂ���
  }
  fopen_s(&file, "test1_Bridge.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);

  wagner.DeleteNode(node3);
  fopen_s(&file, "test1_Delete.txt", "w");
  wagner.GetEdge(file, OutputForGnuplotWithLinesPoints);
  fclose(file);

  return 0;
}

int main() {
  clock_t s = clock();
  printf("test1:");
  test1();
  printf("%dms\n", clock() - s);
  s = clock();
  printf("test2:");
  test2();
  printf("%dms\n", clock() - s);
  s = clock();
  printf("test3:");
  test3();
  printf("%dms\n", clock() - s);
  s = clock();
  printf("test4:");
  test4();
  printf("%dms\n", clock() - s);
  s = clock();
  printf("test5:");
  test5();
  printf("%dms\n", clock() - s);
  return 0;
}
