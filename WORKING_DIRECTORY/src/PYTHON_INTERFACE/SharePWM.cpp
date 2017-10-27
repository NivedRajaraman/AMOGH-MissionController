/*

  Meant as a helper program for Simulator.py
  Passes on the PWM values into a shared memory


 */
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include "state.hpp"
#include <cmath>


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

    if ((shmid = shmget(key, 88, 0666)) < 0) {
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


    //// share pwm values to memory
    int *ptr,*pwm;
    void *shmad;
    key_t key1=0;
    key1=9000;
    if ((shmid = shmget(key1, 24, IPC_CREAT | 0666)) < 0) {
      perror("shmget");
    }
    if ((shmad = (void *)shmat(shmid, NULL, 0)) == (void *) -1) {
      perror("shmat");
    }
    
    ptr = new (shmad) int();
    pwm = new int();
    while (1){
      pwm = new int();
      shm->return_PWM_Side(pwm);
      *ptr = *pwm;
      ptr+=1;
      *ptr = *(pwm+1);
      ptr+=1;
      shm->return_PWM_Back(pwm);
      *ptr = *pwm;
      ptr+=1;
      *ptr = *(pwm+1);
      ptr+=1;
      shm->return_PWM_Bottom(pwm);
      *ptr = *pwm;
      ptr+=1;
      *ptr = *(pwm+1);
      ptr-=5;
      delete pwm;
    }
    
}
