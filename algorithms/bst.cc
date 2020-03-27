#include <stdio.h>
#include <string.h>

int is_same_bst(char * bst, char * bst_cmp, int len) {
	
	if (bst[0] != bst_cmp[0]) 
			return 0;
	
	if (len <= 1 || (len == 2 && bst[1]== bst_cmp[1]))
	{
		return 1;
		}
	
	// assign storage space
	char bst_left[10] = "", bst_right[10] = "",
		 bst_cmp_left[10] = "", bst_cmp_right[10] = "";
	
	int br = 0, bl = 0, bcr = 0, bcl = 0;
	
	for (int i = 1; i < len; i++) {
	bst[i] > bst[0] ? (bst_right[br++] = bst[i]) 
		: (bst_left[bl++] = bst[i]);  
	bst_cmp[i] > bst_cmp[0] ? (bst_cmp_right[bcr++] = bst_cmp[i])
		: (bst_cmp_left[bcl++] = bst_cmp[i]);
	}
	
	return is_same_bst(bst_left, bst_cmp_left, bl) && 
		is_same_bst(bst_right, bst_cmp_right, br);
				
	
}


int main()
{
	int count;
	char bst[30];
	char bst_cmp[30];
	//scanf("%d", &count);
	
	while (~scanf("%d", &count) && count) {
		scanf("%s", bst);	
		while (count--){
			printf("count=%d\n",count);
		scanf("%s", bst_cmp);
	
		printf("%s\n",is_same_bst(bst, bst_cmp, strlen(bst)) ? "YES" : "NO");
	}
	}

	return 0;
}

// char str[10] = "";
// char str[10] = "abcdefg";
// char str[10] = {"abcdefg"};
// char str[10] = "0";

