#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

#define NUM_THREADS  3
#define BUFSIZE 1024
// ===============================
// ===      RECEIVED DATA      ===
// ===============================
typedef struct thread_data{
   int  thread_id;
   float yaw,
         pitch,
         roll;
}t_data;
// data[0]->passed data
// data[1]->data to be passed
t_data data[2];
void setParameter(t_data*,t_data*);

// ===============================
// ===  MUTEX AND CONDITIONAL  ===
// ===============================
pthread_mutex_t conn_mutex;
pthread_mutex_t newval_mutex;
pthread_mutex_t analysing_mutex;

pthread_cond_t conn_cv;
pthread_cond_t newval_cv;
pthread_cond_t analysing_cv;

bool connected = false;
bool newval = false;
bool analysing = false;
void srv_connected();
void newval_recv();
void f_analysing();


// ===============================
// ===          TCP            ===
// ===============================

char *hostname = "192.168.1.28";
int portno = 2222;
int btod(long int n);
int power(int i) ;

// ===============================
// ===        THREADING        ===
// ===============================
int  thread_ids[3] = {0,1,2};

void *tcp_srv(void*);
void *imp_srv(void*);

// ===============================
// ===      MULTIPURPOSE       ===
// ===============================
void error(char*);
// data extraction and selection
void extract(char*,t_data*);
void setParameter(t_data*,t_data*);


// ===============================
// ===         MAIN            ===
// ===============================
#define DEBUG 1
int main (int argc, char *argv[])
{
  int i;
  long t1=1, t2=2, t3=3;
  pthread_t threads[3];
  pthread_attr_t attr;

  /* Initialize mutex and condition variable objects */
  pthread_mutex_init(&conn_mutex, NULL);
  pthread_cond_init (&conn_cv, NULL);
  pthread_mutex_init(&analysing_mutex, NULL);
  pthread_cond_init (&analysing_cv, NULL);
  pthread_mutex_init(&newval_mutex, NULL);
  pthread_cond_init (&newval_cv, NULL);

  /* For portability, explicitly create threads in a joinable state */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&threads[0], &attr, tcp_srv, (void *)t1);
  #ifdef DEBUG
    printf("created tcp_srv()thread\n");
  #endif // DEBUG
  pthread_create(&threads[1], &attr, imp_srv, (void *)t2);
  #ifdef DEBUG
    printf("created imp_srv() thread\n");
  #endif // DEBUG
  /* Wait for all threads to complete */
  for (i=0; i<NUM_THREADS; i++) {
        printf("Back into main()%d\n",i);
    pthread_join(threads[i], NULL);
  }
  #ifdef DEBUG
  printf ("Main(): Waited on %d  threads. Done.\n", NUM_THREADS);
  #endif // DEBUG
  scanf("%d",&i);
  /* Clean up and exit */
  pthread_attr_destroy(&attr);
  pthread_mutex_destroy(&analysing_mutex);
  pthread_mutex_destroy(&conn_mutex);
  pthread_mutex_destroy(&newval_mutex);
  pthread_cond_destroy(&analysing_cv);
  pthread_cond_destroy(&conn_cv);
  pthread_cond_destroy(&newval_cv);
  pthread_exit(NULL);


}

void *tcp_srv(void *t)
{
  int i,n;
  long my_id = (long)t;
  int sockfd, newsockfd;
  int clientLen;

  struct sockaddr_in srv_addr;
  struct sockaddr_in cli_addr;
  struct hostent *hostp;

  char buff[BUFSIZE];
  char *hostaddrp;

  #ifdef DEBUG
    printf("tcp_srv(): inside thread\n");
  #endif // DEBUG
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0)
    error("Error opening socket.\n");
    #ifdef DEBUG
        printf("tcp_srv(): created socket\n");
    #endif // DEBUG

   memset(&srv_addr, 0, sizeof(srv_addr));

  srv_addr.sin_family = AF_INET;
  srv_addr.sin_addr.s_addr = INADDR_ANY;
  srv_addr.sin_port = htons(portno);

  /* Bind socket to port */
  if(bind(sockfd, (struct sockaddr*) &srv_addr, sizeof(srv_addr)) < 0) {
    error("Error on binding.");
    exit(-1);
  }

  /* Now start listening for clients, sleep until client found */
  listen(sockfd, 5);

  #ifdef DEBUG
    printf("tcp_srv() binded socket. listening\n");
  #endif // DEBUG
  clientLen = sizeof(cli_addr);
  newsockfd = accept(sockfd, (struct sockaddr*) &cli_addr, &clientLen);
  if(newsockfd < 0) {
    error("Error on accepting socket.");
    exit(-1);
  }else{
    #ifdef DEBUG
        printf("tcp_srv() accepted connection. new socket created.condition reached\n");
    #endif // DEBUG
    srv_connected();
  }
  #ifdef DEBUG
    printf("tcp_srv(): before while\n");
  #endif // DEBUG

  while(1) {
        #ifdef DEBUG
        printf("tcp_srv(): inside main loop\n");
        #endif // DEBUG
		/* Receive datagram from client */
		bzero(buff, BUFSIZE);
		n = read(newsockfd, buff, BUFSIZE);

		if(n < 0)
			error("Error recvfrom().");
        else {



            while(analysing){
                #ifdef DEBUG
                   printf("tcp_srv(): BLOCKED, data[1] in use\n");
                #endif // DEBU
                 pthread_cond_wait(&analysing_cv,&analysing_mutex);
            }
            #ifdef DEBUG
               printf("tcp_srv(): received %s\n",buff);
            #endif

            analysing=true;
 pthread_mutex_lock(&analysing_mutex);
            extract(buff,&data[1]);
 pthread_mutex_unlock(&analysing_mutex);
            newval_recv();

        }

	}

  pthread_exit(NULL);
}

void *imp_srv(void* t){
  long my_id = (long)t;
  pthread_mutex_lock(&conn_mutex);
  while(!connected){
        #ifdef DEBUG
    printf("impr_srv(): BLOCKED WAITING FOR CONNECTION\n");
        #endif
    pthread_cond_wait(&conn_cv,&conn_mutex);
  }
  while(connected){
        #ifdef DEBUG
        printf("imp_srv(): entered while connected loop\n");
        #endif // DEBUG

      while(!newval){
        #ifdef DEBUG
        printf("imp_srv: BLOCKED WAITING FOR NEWVAL\n");
        #endif // DEBUG
        pthread_cond_wait(&newval_cv,&newval_mutex);
      }

      #ifdef DEBUG
      printf("imp_srv(): yaw-%f\tpitch-%f\troll-%f\n",data[1].yaw,data[1].pitch,data[1].roll);
      #endif
        setParameter(&data[0],&data[1]);
        f_analysing();

        //unblock conditional variable for JUCE API to output the
        newval=false;
  }

}

void f_analysing(){
  pthread_mutex_lock(&analysing_mutex);
  analysing = false;
  pthread_cond_signal(&analysing_cv);
  pthread_mutex_unlock(&analysing_mutex);
}

void srv_connected(){
  pthread_mutex_lock(&conn_mutex);
  connected = true;
  pthread_cond_signal(&conn_cv);
  pthread_mutex_unlock(&conn_mutex);
}


void newval_recv(){
  pthread_mutex_lock(&newval_mutex);
  newval = true;
  pthread_cond_signal(&newval_cv);
  pthread_mutex_unlock(&newval_mutex);
}

void extract(char* buff,t_data* data){
    printf("INSIDE EXTRACT\n");
    int t;
    float f1,f2,f3;
    t=sscanf(buff,"yaw:\t%f\npitch:\t%f\nroll:\t%f",&f1,&f2,&f3);

    if(t!=3){ //number of elements to fill
        printf("%d\t tcp_srv:condition faile still reading data received data:\nyaw-%f\tpitch-%f\troll-%f",data->yaw,data->pitch,data->roll);
        error("tcp_srv: extract failed");
    }else{

    data->yaw = f1;
    data->pitch = f2;
    data ->roll =f3;

    #ifdef DEBUG
        printf("tcp_srv: received data:yaw-%f\tpitch-%f\troll-%f",data->yaw,data->pitch,data->roll);
    #endif // DEBUG
    }
}

void setParameter(t_data* passed,t_data* topass){
    float maxDelta=0;
    int   pos=0,
          i=0;
printf("AM NOW IN HERE\n");
        if(abs(passed->yaw-topass->yaw)){
           maxDelta=passed->yaw-topass->yaw;
           printf("MAX DELTA needs to be ASSIGNED TO YAW\n");
            pos=0;
        }else if(abs(passed->pitch-topass->pitch)){
            maxDelta=passed->pitch-topass->pitch;
            printf("MAX DELTA needs to be ASSIGNED TO PITCH\n");
            pos=1;
        }else{
            maxDelta=passed->roll-topass->roll;
            printf("MAX DELTA needs to be ASSIGNED TO ROLL\n");
            pos=2;
        }
    switch (pos){
        case 0: {passed->yaw=topass->yaw;
                 printf("MAX DELTA ASSIGNED TO YAW\n");
                 break;}
        case 1:{passed->pitch=topass->pitch;
                printf("MAX DELTA ASSIGNED TO PITCH\n");
                break;}
        case 2:{passed->roll=topass->roll;
                printf("MAX DELTA ASSIGNED TO ROLL\n");
                break;}
    }

}


void error(char *msg) {
    char chara;
	perror(msg);
	scanf("%c",&chara);
	exit(1);
}

int btod(long int n) {
	int x, s = 0, i = 0, flag = 1;
	while(flag == 1) {
		x = n % 10;
		s = s + x * power(i);
		i = i + 1;
		n = n / 10;
		if(n == 0)
			flag = 0;
	}
	return s;
}

int power(int i) {
	int j, p = 1;

	for(j = 1; j <= i; j++)
		p = p * 2;

	return p;
}

