#include <unistd.h>
#include <stdio.h>

#include <syscalls.h>

int main()
{
	for (int i = 0; i < 1000; i += 1)
		for (int j = 0; j < 1000; j += 1)
			printf("PAT PAT PAT %d + %d = %d\n", i, j, syscallSummation(i, j));
	
	return 0; 
}
