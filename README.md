# wagner
wagner's theorem triangle
First, you need to know "Wagner's theorem for Torus graphs".
This lib is used the theorem. And we can get triangles with high versatility.
Next, if you want to know this lib, see "WagnerTorusGraphsTest". It is not main code. Sorry for the no refactoring.
Have prepared five tests.
As well as in the source code, the output result file and the image created by gnuplot are prepared with the same name.

test1 : Basic. how to use. Define NodeInfo, RegistNode and get node number, get all edge, bridge edge, delete node.
test2 : Power. Regist 100000 point and bridge.
test3 : Versatility. In the test, get not only xy but z. The condition of A can be changed. In the case of the example, you can use z as the condition.
test4 : get all triangle.
test5 : we can get directed graph.

Finally, if you want to know more, see "WagnerTorusGraphsLib".
Let's think about meshing together.
