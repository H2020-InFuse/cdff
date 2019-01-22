## Valgrind

Valgrind is a tool to help check for memory leaks.

## Jobs

The [partial-valgrind](./partial-valgrind.sh) script runs a full leak check, without reporting conditional jumps on unitialised values of variables.
The [full-valgrind](./full-valgrind.sh) script runs a full leak check, with additionally tracing unitialised values of variables. The latter check takes significantly more time to complete, but can help debugging by providing the specific origin of where the variables where declared.

## How to interpret results

Easiest way to go over the results is by using valkyrie on a Linux system. (`sudo apt install valkyrie')
Some memory leaks cause cascaded errors. So concentrate on the 'easy' and obvious ones to resolve, this might in turn resolve a number of errors that seemed to be false-positives.
