# wagner
wagner's theorem triangle  
First, you need to know *"Wagner's theorem for Torus graphs"*.  
This lib used the theorem. And we can get triangles with high versatility.  
Next, if you want to know this lib, see "WagnerTorusGraphsTest". It is not main code. Sorry for the no refactoring.  
Have prepared five tests.  
As well as in the source code, the output result file and the image created by gnuplot are prepared with the same name.  

test1 : Basic. how to use. Define NodeInfo, RegistNode and get node number, get all edge, bridge edge, delete node.  
test2 : Regist 100000 point and bridge.  
test3 : Versatility. In the test, get not only xy but z. The condition of "diagonal transformation" can be changed. In the case of the example, you can use z as the condition.  
test4 : get all triangle. In the case of the example, the sum of the areas is calculated.  
test5 : we can get directed graph.  

Finally, if you want to know more, see "WagnerTorusGraphsLib".
Let's think about meshing together.


2019年あたりにワーグナーの対角変形定理を利用して設計して作成した三角形分割ライブラリ。
平たく言えば点を追加することでいい感じに三角形分割をしてくれる。
WagnerTorusGraphsLib\WagnerTorusGraphsTestのWtest2_10000を見ればだいたい雰囲気掴めるはず。
図にもある通り特定の線分を強引に結んだりもできるので、例えば多角形の三角形分割とかもやろうと思えばできる。
test3_xyは綺麗に分割してたものを、test3_xyzみたいにルールを変えた三角形分割もできる。途中で点は追加できるし、後から削除もできる。
概ね綺麗に分割する場合、N個の点を追加する場合最悪O(N^2)かかるが、真ん中から追加することで対角変形の回数が減るので速くなる。
やってることはだいたい下記の通りだった気がする
test1 : ライブラリの簡単な使い方
test2 : 大量データでも十分動くお話
test3 : Y座標に凄く大きい重みを添えた場合の三角形分割を考える。平たく言えば図は「Y方向急斜にすごく傾いた紙を上から見た」もの
test4 : 図はないが点群を与えることによってできる凸包の面積を求める
test5 : 一番左下から右上まで向かうことのできるパスを取得する。残念ながら最短経路ではない。
