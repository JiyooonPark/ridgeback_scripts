## Ridgeback + iiwa communication system

|#| Ridgeback | iiwa | Extra |
|---| ----------- | ----------- | ----------- |
| 1 | | | input: picture<br>output: stippled points file <br> | 
| 2 | | input: stippled point file <br> output: farmost left, right (y) point location<br> generates drawing wall coordinates transforms it into ridgeback world coordinates | |
| 3 | input: farmost y drawing coordinates <br> output: list of length of each drawing section, position list<br> run trajectory planning code and generate path  | | |
| 4 | |input: drawing length of each location, position list<br> convert each section drawing instruction to start from (0,0)| |
| 5 | output: move done message <br> move to initial position | | | |
| 6 | input: move done message <br> output: drawing done message <br> | |

## message used

|#| message name | message type | content |
|---|-----| ----------- | ----------- | 
|2 |? |float [2] | left y, right y |
|3 | ?|float list1<br> float list2 | length of each segment<br> ridgeback positions|
|4-inf |? | int| 0 if waiting for other robot to execute <br> # if executing instruction|
