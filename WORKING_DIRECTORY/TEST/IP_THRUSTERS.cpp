#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include "state.hpp"
#include <cmath>

//#include <other headers>
#define SHMSZ 27

main()
{
    int shmid;
    key_t key;
    STATE* shm;

    /*
     * We need to get the segment named
     * "5678", created by the server, i.e. missioncontroller.
     */
    key = 5678;

    /*
     * Locate the segment.
     */

    printf("Shared memory being located...\n");

    if ((shmid = shmget(key, SHMSZ, 0666)) < 0) {
        perror("shmget");
        exit(1);
    }

    printf("Shared memory segment found!\n");
    /*
     * Now we attach the segment to our data space.
     */

    printf("Attaching to data space...\n");
    if ((shm = (STATE *)shmat(shmid, NULL, 0)) == (STATE *) -1) {
        perror("shmat");
        exit(1);
    }

        printf("Successfully initialized...\n");
    
    STATE* shmIP;        //object that copies the value of the shared memory into it (shallow copy)

    shmIP = shm;         //shallow copy

//----------------------------------------------------------------------------------------------------------------------------------
                                                    //END HERE
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------


    shmIP->set_r(350);
    shmIP->set_theta(45);
    shmIP->set_phi(45);
    shmIP->set_pressure(0);
    shmIP->set_timeDiff(0.01);
    shmIP->set_taskID('p');


    //printf("%f\n",  shmIP->return_r());

    int PWM[2];

    FILE* fp;
    fp = fopen("dumpvars_tab.txt", "w");

    float r_0 = 350;
    float theta = 45;
    float phi = 45;

    while(1)            //main loop that recomputes R, theta, phi for different frames
    {
        //shmIP->return_PWM_Bottom(PWM);

        //shmIP->return_PWM_Bottom(PWM);
        //printf("PWM values are: %d\n", *PWM);

        //int speed = (PWM[0] - 1500);
        //printf("Speed is: %d\n", speed);
        //shmIP->set_r(150-speed*0.1);
        
        printf("New r, %f   New theta, %f   New phi, %f \n", shmIP->return_r(), shmIP->return_theta(), shmIP->return_phi());

        shmIP->return_PWM_Side(PWM);
        int speedX = PWM[0] - 1500;

        shmIP->return_PWM_Back(PWM);
        int speedZ = ( PWM[1] + PWM[0] )/2 - 1500;
        int sway = ( PWM[0] - PWM[1] )/10;
        printf("sway speed: %d", sway);
        
        r_0 = r_0-(0.5*pow(pow(speedX, 2) + pow(speedZ, 2),0.5));
        phi = phi - sway;
        
        shmIP->set_r(r_0);
        shmIP->set_phi(phi);

        //printf("delay\n");
        usleep( 50000 );
        //printf("updating file\n");
        fprintf(fp, "%f %f %f\n", shmIP->return_r(), shmIP->return_theta(), shmIP->return_phi());
        fflush(fp);

        //if (shmIP->return_r() < 5) {break;}

    }
    fclose(fp);
    exit(0);
}
