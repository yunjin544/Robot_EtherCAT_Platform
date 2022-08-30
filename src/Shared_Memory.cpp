
#include "Shared_Memory.h"

int *shmem[SHARED_SIZE];
int shmid[SHARED_SIZE] ;
int semid[SHARED_SIZE];
union semun sem_union[SHARED_SIZE];

int shmem_init()
{
    void *memory_segment = NULL;
    

    for(char i=0; i<SHARED_SIZE; i++)
    {
        if((shmid[i] = shmget(KEY_NUM+i,sizeof(char),IPC_CREAT|0666))==-1)
        {
            printf("Shared Memory init - Failed1\n");
        }
        if((memory_segment = shmat(shmid[i],NULL,0)) == (void*)-1) return -1;

        shmem[i] = (int*)(memory_segment);
    }

    return 1;
}

int shmem_clear(){
    
    void *memory_segment = NULL;

    for(char i=0; i<SHARED_SIZE; i++)
    {
        (shmid[i] = shmget(KEY_NUM+i,sizeof(char),IPC_CREAT|0666));
        if (shmctl(shmid[i],IPC_RMID,NULL)==-1) return -1;
    }
    return 0;

}

int semp_init(){

    for(char i=0; i<SHARED_SIZE; i++)
    {
        if(semid[i] = semget(KEY_NUM+i,sizeof(char),IPC_CREAT|0666)==-1)
        {
            if((semid[i] = semget(KEY_NUM+i,0,0))==-1) return -1;
        }
        else{
        sem_union[i].val = 1;
        semctl(semid[i],0,SETVAL,sem_union[i]);
    }
    }
    return 0;

}

void Exit_EventHandler(int sig)
{
    signal(sig,SIG_IGN);
    shmem_clear();

    exit(1);
}

void s_wait(int semid){
    struct sembuf buf;
    buf.sem_num = 0;
    buf.sem_op = -1;
    buf.sem_flg = SEM_UNDO;

    if(semop(semid ,&buf,1)==-1){
        printf("<s_wait> Sempo error ! \n");
        return ;
    }
    
}

void s_quit(int semid){
    struct sembuf buf ;
    buf.sem_num =0;
    buf.sem_op = 1 ;
    buf.sem_flg = SEM_UNDO;

    if(semop(semid ,&buf,1)==-1){
    printf("<s_wait> Sempo error ! \n");
    return ;
    }
    
}