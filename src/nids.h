/*
  Copyright (c) 1999 Rafal Wojtczuk <nergal@7bulls.com>. All rights reserved.
  See the file COPYING for license details.
*/

#ifndef _NIDS_NIDS_H
# define _NIDS_NIDS_H

# include <sys/types.h>
#include <netinet/in_systm.h>
#include <netinet/in.h>
# include <netinet/ip.h>
# include <netinet/tcp.h>
# include <pcap.h>


//add: 2014 1 25   1
#include<pthread.h>
#include<unistd.h>
#include<string.h>
#include<sys/sem.h>
#include<semaphore.h>

sem_t sem_full;
sem_t  sem_empty;
pthread_t th1,th2;
int thread_error;
void * thread_pros1(void *arg);
void *thread_pros2(void *arg);
void *tret;
//end add


# ifdef __cplusplus
extern "C" {
# endif

# define NIDS_MAJOR 1
# define NIDS_MINOR 25

# define CPU_SET(cpu, cpusetp)   __CPU_SET_S (cpu, sizeof (cpu_set_t), cpusetp)  
# define CPU_CLR(cpu, cpusetp)   __CPU_CLR_S (cpu, sizeof (cpu_set_t), cpusetp)  
# define CPU_ISSET(cpu, cpusetp) __CPU_ISSET_S (cpu, sizeof (cpu_set_t),cpusetp)  
# define CPU_ZERO(cpusetp)   __CPU_ZERO_S (sizeof (cpu_set_t), cpusetp)      
   #define __USE_GNU
#include <sched.h> 



enum
{
  NIDS_WARN_IP = 1,
  NIDS_WARN_TCP,
  NIDS_WARN_UDP,
  NIDS_WARN_SCAN
};

enum
{
  NIDS_WARN_UNDEFINED = 0,
  NIDS_WARN_IP_OVERSIZED,
  NIDS_WARN_IP_INVLIST,
  NIDS_WARN_IP_OVERLAP,
  NIDS_WARN_IP_HDR,
  NIDS_WARN_IP_SRR,
  NIDS_WARN_TCP_TOOMUCH,
  NIDS_WARN_TCP_HDR,
  NIDS_WARN_TCP_BIGQUEUE,
  NIDS_WARN_TCP_BADFLAGS
};

# define NIDS_JUST_EST 1
# define NIDS_DATA 2
# define NIDS_CLOSE 3
# define NIDS_RESET 4
# define NIDS_TIMED_OUT 5
# define NIDS_EXITING   6	/* nids is exiting; last chance to get data */

# define NIDS_DO_CHKSUM  0
# define NIDS_DONT_CHKSUM 1

struct tuple4
{
  u_short source;
  u_short dest;
  u_int saddr;
  u_int daddr;
};

struct half_stream
{
  // ��¼�ð����ӵ�״̬������ΪFIN_SENT��
  char state;
  // ���û�ע��ĺ��������޸ģ�����libnids�Ƿ���Ҫ��������tcp����
  char collect;
  // ���û�ע��ĺ��������޸ģ�����libnids�Ƿ���Ҫ��������tcp����
  char collect_urg;

  // ָ��һ�����棬�û��汣�����ѱ�ȷ�ϵ�tcp�����ֽ�����
  char *data;
  // count - offset = ��ǰdata�е����ݳ���
  int offset;
  int count;
  // �����յ��ı��ĵĴ�С
  int count_new;
  // ��¼data��ָ����ڴ��ʵ�ʴ�С
  int bufsize;
  // ��¼list����ռ�õ��ܴ�С
  int rmem_alloc;

  // 
  int urg_count;
  u_int acked;
  u_int seq;
  u_int ack_seq;
  // �������ӵ�һ���ֽ����
  u_int first_data_seq;
  u_char urgdata;
  // һ����־����notify�����л�ʹ�õ�����¼�Ƿ��յ���һ���µĽ�������
  u_char count_new_urg;
  // һ����־����¼���������Ƿ񿴵��˽�������
  // ���Ϊ0��ʾû�п��������Ϊ1��ʾ�н�������û�д���
  u_char urg_seen;
  // ָ��ո��յ��ı��ĵĽ������ݵ���ʼ��ַ
  u_int urg_ptr;
  u_short window;
  // ��¼�Ƿ�����ʱ���
  u_char ts_on;
  // ��¼�Ƿ�ʹ�ô��ڴ�С
  u_char wscale_on;
  // ��ǰʱ���
  u_int curr_ts;
  // ���ڴ�С
  u_int wscale;

  // ָ��һ�������������е�ÿһ���ڵ㶼��һ��tcp����
  // ��Щtcp�����ǽ������Ѿ����յ�����û��ȷ�ϵı���
  struct skbuff *list;
  // ������β�������µı����õ�
  struct skbuff *listtail;
};


struct tcp_stream
{
  // ��Ԫ���ַ
  struct tuple4 addr;
  // tcp ��һ��״̬
  char nids_state;
  // �����tcp��ص����м�������
  struct lurker_node *listeners;
  // ������
  struct half_stream client;
  struct half_stream server;
  // ������
  struct tcp_stream *next_node;
  struct tcp_stream *prev_node;
  // ��¼��tcp��tcp���е�hashֵ
  int hash_index;
  // 
  struct tcp_stream *next_time;
  struct tcp_stream *prev_time;
  // ��¼���λص��û���ע�ắ�����ȡ�����ֽڵ�����
  int read;
  // ��һ�����õĿյ�tcp�ڵ�
  struct tcp_stream *next_free;
  void *user;
  // ʱ���
  long ts;
};

struct nids_prm
{
  int n_tcp_streams;
  int n_hosts;
  char *device;
  char *filename;
  int sk_buff_size;
  int dev_addon;
  void (*syslog) ();
  int syslog_level;
  int scan_num_hosts;
  int scan_delay;
  int scan_num_ports;
  void (*no_mem) (char *);
  int (*ip_filter) ();
  char *pcap_filter;
  int promisc;
  int one_loop_less;
  int pcap_timeout;
  int multiproc;
  int queue_limit;
  int tcp_workarounds;
  pcap_t *pcap_desc;
  int tcp_flow_timeout;
};

struct tcp_timeout
{
  // ��¼��ǰ��timeout����������һ��tcp
  struct tcp_stream *a_tcp;
  // 
  struct timeval timeout;
  // �������ÿһ���ڵ��Ӧһ��tcp������ڵ��Ӧ���tcp
  struct tcp_timeout *next;
  struct tcp_timeout *prev;
};

// modified - 2014-01-25
struct fifo_node
{
	struct ip *data;
	int skblen;
};

struct nids_fifo
{
	struct fifo_node * head;
	struct fifo_node * tail;
	struct fifo_node * start;
	struct fifo_node * end;
	int fifo_len;
};
// modified end


int nids_init (void);
void nids_register_ip_frag (void (*));
void nids_unregister_ip_frag (void (*));
void nids_register_ip (void (*));
void nids_unregister_ip (void (*));
void nids_register_tcp (void (*));
void nids_unregister_tcp (void (*x));
void nids_register_udp (void (*));
void nids_unregister_udp (void (*));
void nids_killtcp (struct tcp_stream *);
void nids_discard (struct tcp_stream *, int);
int nids_run (void);
void nids_exit(void);
int nids_getfd (void);
int nids_dispatch (int);
int nids_next (void);
void nids_pcap_handler(u_char *, struct pcap_pkthdr *, u_char *);
struct tcp_stream *nids_find_tcp_stream(struct tuple4 *);
void nids_free_tcp_stream(struct tcp_stream *);

extern struct nids_prm nids_params;
extern char *nids_warnings[];
extern char nids_errbuf[];
extern struct pcap_pkthdr *nids_last_pcap_header;
extern u_char *nids_last_pcap_data;
extern u_int nids_linkoffset;
extern struct tcp_timeout *nids_tcp_timeouts;

struct nids_chksum_ctl {
	u_int netaddr;
	u_int mask;
	u_int action;
	u_int reserved;
};
extern void nids_register_chksum_ctl(struct nids_chksum_ctl *, int);

# ifdef __cplusplus
}
# endif

#endif /* _NIDS_NIDS_H */
