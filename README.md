[fig1]: /images/fig1.png " "
[fig2]: /images/fig2.png " "
[fig3]: /images/fig3.png " "
[fig4]: /images/fig4.png " "
[fig5]: /images/fig5.png " "
[fig6]: /images/fig6.png " "
[fig7]: /images/fig7.png " "

#RRT and Bi-RRT plugin for OPENRAVE

##Code Implementation
The files `RRTNode.h` and `NodeTree.h` were created to contain the classes used in the RRT implementation. The class `NodeTree` received the method `getNearestNeighbor`, which receives as input a configuration vector and returns a pointer to the nearest node to that configuration. This is used in the RRT code to connect the tree to the node.
##Defining RRT Goal Bias
In order to execute the code, the function `searchRRT` must be called with start end goal configuration passed as parameters. Some aditional parameters can be passed too, such as goal bias, display the arm moving while searching (search might take longer), and whether you want bidirectional search. The code was executed with the goal bias ranging from 1% to 96%, ten times at each configuration. The data obtained in this execution was plotted. It can be seen that the lowest point on the figure is at bias 0.06, and it slowly increases until around 0.7, where it starts increasing fastly. The reason for this increase is that with a higher chance of biasing the tree to the goal, it gives less computation time to let the tree grow around obstacles before the goal is selected, leading to several collisions with no successful advance towards the goal.

![alt text][fig1]

##Planned Path

![alt text][fig2]

##Path Smoothing

The shortcut smoothing algorithm was implemented in C++. In order to run the algorithm, the function `smoothPath` should be called in Python, with the number of iterations as parameter. It will take the RRT path calculated in the previous step and will smooth it by clipping the values from the path where a shortcut is not in collision. The resulting path is shown in figure below. As the smooth path is sparse(each point in the path is far from each other), the connection between two points of the path will not reflect accurately the end-effector position, therefore a special function `drawSmooth` was created to interpolate points between each point in the path and create a realistic end-effector trajectory on the screen.

![alt text][fig3]

In the figure below, the length of the path is plotted against the number of times it was executed. Iy can be noted a fast decrease in size on the initial steps, and a steady lenght by the end, as the path reaches a local minimum.

![alt text][fig4]

##Execution Time comparison
The algorithm was executed then 30 times with goal bias 0.06, producing an average execution time of 15.74 seconds, slightly higher than the previous measurement for the same bias, but still within the error margin. The number of samples done in the RRT have a significant variance, because due to the randomness, it doesn’t take the same time to reach the goal, and in general more samples are done in the executions that takes longer to complete. The Mean Squared Error of the data shows that the number of samples done by the RRT and the time executed varies at a similar rate of around 0.25, correlating both measurements.The Bidirectional RRT performed slightly worse in this scenario. This is due to the fact that this narrow passage most probably is not a bugtrap in C-Space, which means that its not harder to find a path from neither side of the narrow passage, therefore using a bi-directional RRT makes it take longer in average to converge to the solution by sampling an almost twice as large samples, as it can be seen by comparing the number of nodes sampled in the simple RRT and the bi-directional RRT. However, if the Histograms for the execution time are taken in consideration( Figures below), it can be seen that at most executions both RRT and Bi-RRT performed in around 20 seconds. Due to the almost twice as large number of samples in the Bi-RRT, usually a finer path was found, leading to a better result after smoothing.
The smoothing algorithm performance seems to be related with the length of the unsmoothed path, as longer paths tend to execute he smoothing for more time. This is probably due to the time taken doing collision checks.

###Execution times and values for RRT and Bi-RRT
![alt text][fig5]

###Time execution Histogram for RRT-Connect execution
![alt text][fig6]
###Time execution Histogram for bi-directional RRT execution
![alt text][fig7]

##Instructions to run Bi-RRT
In order to run Bi-RRT code, when calling the python wrapper function on line 131, `searchRRT`, change the method’s named parameter bidirectional from `False` to `True`.
