#include<stdio.h>

int main()
{
    int count, num;
    scanf("%d", &count);
    
    while (count--) {
		scanf("%d", &num);
		int sum = 0;
		int max_num = 0;
		
		for (int i = 0; i< num; ++i) {
			int suger_count;
			scanf("%d", &suger_count);
			
			sum += suger_count;
			
			if (suger_count > max_num) {
			max_num = suger_count;
			}
		}
		
		printf("%s\n",sum-max_num +1 >= max_num ? "YES" : "NO");
	}

    return 0;
}


