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
struct thread_data{
   int  thread_id;
   float yaw,
         pitch,
         roll;
};
// data[0]->passed data
// data[1]->data to be passed
struct thread_data data[2];
//dummy data variables
int val2pass = 0;
int nval2pass = -1;

// ===============================
// ===  MUTEX AND CONDITIONAL  ===
// ===============================
pthread_mutex_t conn_mutex;
pthread_mutex_t newval_mutex;

pthread_cond_t conn_cv;
pthread_cond_t newval_cv;

bool connected = false;
bool newval = false;

void srv_connected();
void newval_recv();



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

// ===============================
// ===         MAIN            ===
// ===============================

int main (int argc, char *argv[])
{
  int i;
  long t1=1, t2=2, t3=3;
  pthread_t threads[3];
  pthread_attr_t attr;

  /* Initialize mutex and condition variable objects */
  pthread_mutex_init(&conn_mutex, NULL);
  pthread_cond_init (&conn_cv, NULL);
  pthread_mutex_init(&newval_mutex, NULL);
  pthread_cond_init (&newval_cv, NULL);

  /* For portability, explicitly create threads in a joinable state */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_create(&threads[0], &attr, tcp_srv, (void *)t1);
  printf("created tcp_srv()thread\n");
  pthread_create(&threads[1], &attr, imp_srv, (void *)t2);
  printf("created imp_srv() thread\n");

  /* Wait for all threads to complete */
  for (i=0; i<NUM_THREADS; i++) {
        printf("Back into main()%d\n",i);
    pthread_join(threads[i], NULL);
  }
  printf ("Main(): Waited on %d  threads. Done.\n", NUM_THREADS);
  scanf("%d",&i);
  /* Clean up and exit */
  pthread_attr_destroy(&attr);
  pthread_mutex_destroy(&conn_mutex);
  pthread_mutex_destroy(&newval_mutex);
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


  printf("tcp_srv(): inside thread\n");

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0)
    error("Error opening socket.\n");
  printf("tcp_srv(): created socket\n");

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


  printf("tcp_srv() binded socket. listening\n");
  clientLen = sizeof(cli_addr);
  newsockfd = accept(sockfd, (struct sockaddr*) &cli_addr, &clientLen);
  if(newsockfd < 0) {
    error("Error on accepting socket.");
    exit(-1);
  }else{
    printf("tcp_srv() accepted connection. new socket created.condition reached\n");
    srv_connected();
  }
printf("tcp_srv(): before while\n");
  while(1) {

		/* Receive datagram from client */
		bzero(buff, BUFSIZE);
		n = read(newsockfd, buff, BUFSIZE);

		if(n < 0)
			error("Error recvfrom().");
        else printf("tcp_srv(): %s\n",buff);

		/*check if a different value was received
		nval2pass = (int) btod(atol(buff));
		if (nval2pass != val2pass) {
            val2pass=nval2pass;
            // if so notify imp_srv() thread
            newval_recv();
            printf("tcp_srv(): thread %ld, assigned value %d to val2pass\n",
                my_id, val2pass);
        }else{
            printf("tcp_srv(): thread %ld, nval2pass=val2pass= %d\n",
                my_id, val2pass);

        }*/
	}

  pthread_exit(NULL);
}

void *imp_srv(void* t){
  long my_id = (long)t;
  pthread_mutex_lock(&conn_mutex);
  while(!connected){
    printf("impr_srv(): BLOCKED WAITING FOR CONNECTION\n");
    pthread_cond_wait(&conn_cv,&conn_mutex);
  }
  while(connected){
        printf("imp_srv(): entered while connected loop\n");
      while(!newval){
        pthread_cond_wait(&newval_cv,&newval_mutex);
        printf("imp_srv: BLOCKED WAITING FOR NEWVAL\n");
      }
      printf("imp_srv(): thread %ld, nval2pass=val2pass= %d\n",
                    my_id, val2pass);
        //add data interpretation here
        //unblock conditional variable for JUCE API to output the
        newval=false;
  }

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

