
/*
 * Author: Dr.Sam Siewert
 * Modified by: Sundar Krishnakumar
 * Brief: The code performs N&S tests: Completion Time Test,
 *        Scheduling Point Test to check if a task set is feasible 
 *        for scheduling using RM policy 
 *         
 *        This code tests the standard test cases only
 * 
 */


#include <math.h>
#include <stdio.h>

#define TRUE 1
#define FALSE 0
#define U32_T unsigned int

// 1hz

U32_T ex0_period[] = {10, 40, 40};
U32_T ex0_wcet[] = {1, 1, 17};

// 5hz
U32_T ex1_period[] = {10, 40, 40};
U32_T ex1_wcet[] = {1, 1, 4};

// 10hz

U32_T ex2_period[] = {8, 33, 33};
U32_T ex2_wcet[] = {1, 1, 4};



int completion_time_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);
int scheduling_point_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[]);


int main(void)
{ 
    int i;
	U32_T numServices;
    
    printf("******** Completion Test Feasibility Example\n");
   
    printf("Ex-0 U=%4.2f (C1=1, C2=1, C3=17; T1=10, T2=40, T3=40; T=D): ",
		   ((1.0/10.0) + (1.0/40.0) + (17.0/40.0)));
	numServices = sizeof(ex0_period)/sizeof(U32_T);
    if(completion_time_feasibility(numServices, ex0_period, ex0_wcet, ex0_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");

    printf("Ex-1 U=%4.2f (C1=1, C2=1, C3=4; T1=10, T2=40, T3=40; T=D): ",
		   ((1.0/10.0) + (1.0/40.0) + (4.0/40.0)));
	numServices = sizeof(ex0_period)/sizeof(U32_T);
    if(completion_time_feasibility(numServices, ex1_period, ex1_wcet, ex1_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");

    printf("Ex-2 U=%4.2f (C1=1, C2=1, C3=4; T1=8, T2=33, T3=33; T=D): ",
		   ((1.0/8.0) + (1.0/33.0) + (4.0/33.0)));
	numServices = sizeof(ex0_period)/sizeof(U32_T);
    if(completion_time_feasibility(numServices, ex2_period, ex2_wcet, ex2_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");



	printf("\n\n");
    printf("******** Scheduling Point Feasibility Example\n");

    // printf("Ex-0 U=%4.2f (C1=1, C2=1, C3=2; T1=2, T2=10, T3=15; T=D): ",
	// 	   ((1.0/6.0) + (2.0/8.0) + (4.0/12.0) + (6.0/24.0)));
	// numServices = sizeof(ex0_period)/sizeof(U32_T);
    // if(scheduling_point_feasibility(numServices, ex0_period, ex0_wcet, ex0_period) == TRUE)
    //     printf("FEASIBLE\n");
    // else
    //     printf("INFEASIBLE\n");


    printf("Ex-0 U=%4.2f (C1=1, C2=1, C3=17; T1=10, T2=40, T3=40; T=D): ",
		   ((1.0/10.0) + (1.0/40.0) + (17.0/40.0)));
	numServices = sizeof(ex0_period)/sizeof(U32_T);
    if(scheduling_point_feasibility(numServices, ex0_period, ex0_wcet, ex0_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");

    printf("Ex-1 U=%4.2f (C1=1, C2=1, C3=4; T1=10, T2=40, T3=40; T=D): ",
		   ((1.0/10.0) + (1.0/40.0) + (4.0/40.0)));
	numServices = sizeof(ex0_period)/sizeof(U32_T);
    if(scheduling_point_feasibility(numServices, ex1_period, ex1_wcet, ex1_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");

    printf("Ex-2 U=%4.2f (C1=1, C2=1, C3=4; T1=8, T2=33, T3=33; T=D): ",
		   ((1.0/8.0) + (1.0/33.0) + (4.0/33.0)));
	numServices = sizeof(ex0_period)/sizeof(U32_T);
    if(scheduling_point_feasibility(numServices, ex2_period, ex2_wcet, ex2_period) == TRUE)
        printf("FEASIBLE\n");
    else
        printf("INFEASIBLE\n");


}


int completion_time_feasibility(U32_T numServices, U32_T period[], U32_T wcet[], U32_T deadline[])
{
  int i, j;
  U32_T an, anext;
  
  // assume feasible until we find otherwise
  int set_feasible=TRUE;
   
  //printf("numServices=%d\n", numServices);
  
  for (i=0; i < numServices; i++)
  {
       an=0; anext=0;
       
       for (j=0; j <= i; j++)
       {
           an+=wcet[j];
       }
       
	   //printf("i=%d, an=%d\n", i, an);

       while(1)
       {
             anext=wcet[i];
	     
             for (j=0; j < i; j++)
                 anext += ceil(((double)an)/((double)period[j]))*wcet[j];
		 
             if (anext == an)
                break;
             else
                an=anext;

			 //printf("an=%d, anext=%d\n", an, anext);
       }
       
	   //printf("an=%d, deadline[%d]=%d\n", an, i, deadline[i]);

       if (an > deadline[i])
       {
          set_feasible=FALSE;
       }
  }
  
  return set_feasible;
}


int scheduling_point_feasibility(U32_T numServices, U32_T period[], 
								 U32_T wcet[], U32_T deadline[])
{
   int rc = TRUE, i, j, k, l, status, temp;

   for (i=0; i < numServices; i++) // iterate from highest to lowest priority
   {
      status=0;

      for (k=0; k<=i; k++) 
      {
          for (l=1; l <= (floor((double)period[i]/(double)period[k])); l++)
          {
               temp=0;

               for (j=0; j<=i; j++) temp += wcet[j] * ceil((double)l*(double)period[k]/(double)period[j]);

               if (temp <= (l*period[k]))
			   {
				   status=1;
				   break;
			   }
           }
           if (status) break;
      }
      if (!status) rc=FALSE;
   }
   return rc;
}
