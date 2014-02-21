/*
  This file is taken from Linux 2.0.36 kernel source.
  Modified in Jun 99 by Nergal.
*/

#include <config.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "checksum.h"
#include "ip_fragment.h"
#include "tcp.h"
#include "util.h"
#include "nids.h"

#define IP_CE		0x8000	/* Flag: "Congestion" */
#define IP_DF		0x4000	/* Flag: "Don't Fragment" */
#define IP_MF		0x2000	/* Flag: "More Fragments" */
#define IP_OFFSET	0x1FFF	/* "Fragment Offset" part */

#define IP_FRAG_TIME	(30 * 1000)	/* fragment lifetime */

#define UNUSED 314159
#define FREE_READ UNUSED
#define FREE_WRITE UNUSED
#define GFP_ATOMIC UNUSED
#define NETDEBUG(x)

struct sk_buff
{
	char *data;
	int truesize;
};

struct timer_list
{
	struct timer_list *prev;
	struct timer_list *next;
	int expires;
	// ����ص���������ʼ��Ϊ ip_expire
	void (*function)();
	// data��ָ�� struct ipq ��һ��ָ�룬�ں���Ĵ����п��Կ���
	unsigned long data;
	// struct ipq *frags;
};

// ����ͬһ��ip������fragments
// ����ṹ�������� struct ip *ipqueue
struct hostfrags
{
	// ��ָ�����hostfrag��������ipqueue
	struct ipq *ipqueue;
	int ip_frag_mem;
	u_int ip;
	int hash_index;
	struct hostfrags *prev;
	struct hostfrags *next;
};

/* Describe an IP fragment. */
// ����һ��ip��Ƭ����Ҫ�ṹ��sk_buff
struct ipfrag
{
	int offset;			/* offset of fragment in IP datagram    */
	int end;			/* last byte of data in datagram        */
	int len;			/* length of this fragment              */
	struct sk_buff *skb;		/* complete received fragment           */
	unsigned char *ptr;		/* pointer into real fragment data      */
	struct ipfrag *next;		/* linked list pointers                 */
	struct ipfrag *prev;
};

/* Describe an entry in the "incomplete datagrams" queue. */
struct ipq
{
	unsigned char *mac;		/* pointer to MAC header                */
	struct ip *iph;		/* pointer to IP header                 */
	int len;			/* total length of original datagram    */
	short ihlen;			/* length of the IP header              */
	short maclen;			/* length of the MAC header             */
	
	// ָ��һ�������queue��Ӧ��timer_list
	struct timer_list timer;	/* when will this queue expire?         */
	
	// ָ��һ��ipfrag����������ÿһ���ڵ㱣��һ��ip��Ƭ
	struct ipfrag *fragments;	/* linked list of received fragments    */
	
	// ָ��һ��hostfrag����
	struct hostfrags *hf;

	
	struct ipq *next;		/* linked list pointers                 */
	struct ipq *prev;
	// struct device *dev;	/* Device - for icmp replies */
};

/*
  Fragment cache limits. We will commit 256K at one time. Should we
  cross that limit we will prune down to 192K. This should cope with
  even the most extreme cases without allowing an attacker to
  measurably harm machine performance.
*/
#define IPFRAG_HIGH_THRESH		(256*1024)
#define IPFRAG_LOW_THRESH		(192*1024)

/*
  This fragment handler is a bit of a heap. On the other hand it works
  quite happily and handles things quite well.
*/
static struct hostfrags **fragtable;
static struct hostfrags *this_host;
static int numpack = 0;
static int hash_size;
static int timenow;
static unsigned int time0;
static struct timer_list *timer_head = 0, *timer_tail = 0;

#define int_ntoa(x)	inet_ntoa(*((struct in_addr *)&x))


// ���ص�ǰʱ�䣬����Ϊ��λ
static int
jiffies()
{
	struct timeval tv;

	if (timenow)
		return timenow;
	
	// ���timenowΪ0���ִ�������ȡʱ��
	gettimeofday(&tv, 0);
	// ��ת���ɺ���,΢��ת���ɺ���
	timenow = (tv.tv_sec - time0) * 1000 + tv.tv_usec / 1000;

	return timenow;
}


/* Memory Tracking Functions */
// ԭ�Ӳ�����
static void
atomic_sub(int ile, int *co)
{
	*co -= ile;
}

// ԭ�Ӳ�����
static void
atomic_add(int ile, int *co)
{
	*co += ile;
}

// �ͷ�һ��sk_buff�ṹ���С���ڴ�ռ�
static void
kfree_skb(struct sk_buff * skb, int type)
{
	// type������������������Ժ���չ�õģ���һ���汾û���õ�
	(void)type;
	free(skb);
}


// ��ʾ������Ϣ�������Ƴ�
static void
panic(char *str)
{
	fprintf(stderr, "%s", str);
	exit(1);
}


// ����timer_list����������Ϊ����һ��timer�ṹ��
static void
add_timer(struct timer_list * x)
{
	// ���βָ�벻Ϊ��
	if (timer_tail)
	{
		timer_tail->next = x;
		x->prev = timer_tail;
		x->next = 0;
		timer_tail = x;
	}
	else
	{
		x->prev = 0;
		x->next = 0;
		timer_tail = timer_head = x;
	}
}


// ��timer_list������ɾ��������timer
static void
del_timer(struct timer_list * x)
{
	// �����������ͷ
	if (x->prev)
		x->prev->next = x->next;
	// ����������ͷ
	else
		timer_head = x->next;

	// �����������β
	if (x->next)
		x->next->prev = x->prev;
	// ����������β
	else
		timer_tail = x->prev;
}


// ɾ������ͬһ��host������fragments
// �����޸Ķ�Ӧ��host_frag�ṹ�м�¼ip_frag_mem�Ĵ�С��Ȼ���ͷŵ����skb�ṹ��
static void
frag_kfree_skb(struct sk_buff * skb, int type)
{
	if (this_host)
		atomic_sub(skb->truesize, &this_host->ip_frag_mem);
	kfree_skb(skb, type);
}

// �ͷŸ���ָ�뿪ʼ������Ϊlen���ڴ棬�������һ��������
// ��������ܹ������ɵ��ͷ������С���ڴ�顣ǰ������len�����ڴ����ͬһ��malloc�����
static void
frag_kfree_s(void *ptr, int len)
{
	if (this_host)
		atomic_sub(len, &this_host->ip_frag_mem);
	free(ptr);
}


// Ϊ��ǰ���� this_host����һ���ڴ棬Ȼ�󷵻�ָ���ڴ�ĵ�ַ��
// ����ʹ����malloc
static void *
frag_kmalloc(int size, int dummy)
{
	void *vp = (void *) malloc(size);
	// ������������Ժ���չ�á�
	(void)dummy;
	if (!vp)
		return NULL;
	atomic_add(size, &this_host->ip_frag_mem);

	return vp;
}



/* Create a new fragment entry. */
// ����һ��ip��Ƭ�ṹ
// ���������:
//       offset ����Ƭ������ip�����е�ƫ����
//       end    ����Ƭ���һ���ֽ������ݱ��е�λ��
//       skb    ����Ƭ��Ӧ�Ļ���
//       prt    ָ�����Ƭ�ڲ�ĳһ���ֽڵ�ָ�룬���԰��ֽڷ���
//
// 
//  ע��: ���ipfrag�ṹ��û�й��ص�ĳһ��ipq����
static struct ipfrag *
ip_frag_create(int offset, int end, struct sk_buff * skb, unsigned char *ptr)
{
	struct ipfrag *fp;

	// �������malloc����һ��ipfrag�ռ�
	fp = (struct ipfrag *) frag_kmalloc(sizeof(struct ipfrag), GFP_ATOMIC);
	if (fp == NULL)
	{
		// NETDEBUG(printk("IP: frag_create: no memory left !\n"));
		nids_params.no_mem("ip_frag_create");
		return (NULL);
	}

	// ���ڴ����Ϊ0
	memset(fp, 0, sizeof(struct ipfrag));

	/* Fill in the structure. */
	fp->offset = offset;
	fp->end = end;
	fp->len = end - offset;
	fp->skb = skb;
	fp->ptr = ptr;

	/* Charge for the SKB as well. */
	// �ñ�host���ڴ��ֶ�
	this_host->ip_frag_mem += skb->truesize;

	return (fp);
}


// ����hash index
// �����ǣ� ip��Ӧ��������hash���С��ģ
// �����ͬ��ipһ����ӳ�䵽��ͬ��hash�����ϣ����ǲ�ͬ��ipҲ�п���ӳ�䵽��ͬ��hash������
static int
frag_index(struct ip * iph)
{
	unsigned int ip = ntohl(iph->ip_dst.s_addr);

	return (ip % hash_size);
}



// ����������ipͷ���ҵ������ip��Ƭ��Ӧ�����������Ϣ
// ������������Ϣ����ȫ�ֱ���this_host����
// �ɹ�����this_host������1�����򷵻�0�� this_hostҲΪ0
//
// ������������Կ�����ÿһ��host������һ��hash���У�
// ���õ����������hash��ͻ
static int
hostfrag_find(struct ip * iph)
{
	// ��������һ��hash index
	int hash_index = frag_index(iph);
	struct hostfrags *hf;

	// ��ȫ�ֱ�������,���ȫ�ֱ�������ָ��ǰ���Ķ�Ӧ��host
	this_host = 0;
	// 
	for (hf = fragtable[hash_index]; hf; hf = hf->next)
		if (hf->ip == iph->ip_dst.s_addr)
		{
			this_host = hf;
			break;
		}
	// ����Ҳ������򷵻�0�����򷵻�1
	if (!this_host)
		return 0;
	else
		return 1;
}


// 
static void
hostfrag_create(struct ip * iph)
{
	// mknew�������յ�����malloc���ֽ�Ϊ��λ�����ڴ�
	struct hostfrags *hf = mknew(struct hostfrags);
	// ����hash index
	int hash_index = frag_index(iph);

	// ���hostfrags �ṹ��
	hf->prev = 0;
	// ���뵽hash��ͷ
	hf->next = fragtable[hash_index];
	// ά��˫������
	if (hf->next)
		hf->next->prev = hf;
	// �ҵ�hash����
	fragtable[hash_index] = hf;

	// ���ip
	hf->ip = iph->ip_dst.s_addr;
	// ���ݱ����г�ʼ��Ϊ��
	hf->ipqueue = 0;
	// ռ���ڴ��С��ʼ��Ϊ0
	hf->ip_frag_mem = 0;
	hf->hash_index = hash_index;
	// ���õ�ǰhostΪ�ոմ�����host
	this_host = hf;
}


// ɾ����ǰ�ڵ�
static void
rmthis_host()
{
	// ���index
	int hash_index = this_host->hash_index;

	// ��hash ������ժ����
	if (this_host->prev)
	{
		this_host->prev->next = this_host->next;
		if (this_host->next)
			this_host->next->prev = this_host->prev;
	}
	else
	{
		fragtable[hash_index] = this_host->next;
		if (this_host->next)
			this_host->next->prev = 0;
	}
	// �ͷſռ�
	free(this_host);
	// ����Ϊ0
	this_host = 0;
}



/*
  Find the correct entry in the "incomplete datagrams" queue for this
  IP datagram, and return the queue entry address if found.
*/
// ����һ��ipͷ���ӵ�ǰ��host���ҵ������ipͷ��ͬ��ip����
// ���౨�ľ�����ͬ�� 1��ip_id�� 2��ipԴ��Ŀ�ĵ�ַ�� 3��ip�ϲ�Э��
static struct ipq *
ip_find(struct ip * iph)
{
	struct ipq *qp;
	struct ipq *qplast;

	qplast = NULL;
	// �ڵ�ǰhost��Ӧ��queue�б���
	for (qp = this_host->ipqueue; qp != NULL; qplast = qp, qp = qp->next)
	{
		// id��ͬ��Ŀ����Դ��ַ��ͬ���ϲ�Э����ͬ��������
		// ע: ip_id��ip��Ƭ��ʾ������������ip�����
		// �ο�:http://www.360doc.com/content/11/1026/13/7899729_159299493.shtml
		if (iph->ip_id == qp->iph->ip_id &&
		        iph->ip_src.s_addr == qp->iph->ip_src.s_addr &&
		        iph->ip_dst.s_addr == qp->iph->ip_dst.s_addr &&
		        iph->ip_p == qp->iph->ip_p)
		{
			// һ��queueֻ��Ӧһ��timer
			del_timer(&qp->timer);	/* So it doesn't vanish on us. The timer will be reset anyway */
			return (qp);
		}
	}
	return (NULL);
}



/*
  Remove an entry from the "incomplete datagrams" queue, either
  because we completed, reassembled and processed it, or because it
  timed out.
*/
// �ӵ�ǰhost��ɾ����һ��������ip���Ķ���
static void
ip_free(struct ipq * qp)
{
	struct ipfrag *fp;
	struct ipfrag *xp;

	/* Stop the timer for this entry. */
	del_timer(&qp->timer);

	/* Remove this entry from the "incomplete datagrams" queue. */

	// ���������host�еĵ�һ������
	if (qp->prev == NULL)
	{
		this_host->ipqueue = qp->next;
		// ������滹�У����޸ĺ���һ��Ϊ��һ��
		if (this_host->ipqueue != NULL)
			this_host->ipqueue->prev = NULL;
		// �����ɾ���Ķ��������һ����˵��host�Ѿ�û�ж����ˣ�ɾ��host
		else
			rmthis_host();
	}
	// �����ǵ�һ��
	else
	{
		qp->prev->next = qp->next;
		// ����������һ�����޸ĺ�һ����ָ��
		if (qp->next != NULL)
			qp->next->prev = qp->prev;
	}
	
	/* Release all fragment data. */
	// ����һ�����������е�fragmentsȫ���ͷŵ�
	fp = qp->fragments;
	while (fp != NULL)
	{
		xp = fp->next;
		// �ͷ�ipfrag�е�ksb
		frag_kfree_skb(fp->skb, FREE_READ);
		// ���ͷ�ipfrag�ṹ
		frag_kfree_s(fp, sizeof(struct ipfrag));
		fp = xp;
	}
	
	/* Release the IP header. */
	frag_kfree_s(qp->iph, 64 + 8);

	/* Finally, release the queue descriptor itself. */
	frag_kfree_s(qp, sizeof(struct ipq));
}

/* Oops- a fragment queue timed out.  Kill it and send an ICMP reply. */
//  ����:  
//       ��ʱ��ip���е�ָ��
//  ���:
//        ��
//
//  ע��: ��һ��������û�з���icmp������ɾ���˶�Ӧ��ip����
static void
ip_expire(unsigned long arg)
{
	struct ipq *qp;

	qp = (struct ipq *) arg;

	/* Nuke the fragment queue. */
	// �����ip����ɾ����
	ip_free(qp);
}


/*
  Memory limiting on fragments. Evictor trashes the oldest fragment
  queue until we are back under the low threshold.
*/
//
//
//
static void
ip_evictor(void)
{
	// fprintf(stderr, "ip_evict:numpack=%i\n", numpack);

	// �����ǰhost���ڣ����ҵ�ǰhost���ڴ����������
	while (this_host && this_host->ip_frag_mem > IPFRAG_LOW_THRESH)
	{
		// �����Ӧ��ip���в�Ϊ��
		if (!this_host->ipqueue)
			// ����(�����exit)
			panic("ip_evictor: memcount");
		// �ͷŵ�ǰ��ipqueue, һֱ����ǰhost���ڴ�С������
		ip_free(this_host->ipqueue);
	}
}


/*
  Add an entry to the 'ipq' queue for a newly received IP datagram.
  We will (hopefully :-) receive all other fragments of this datagram
  in time, so we just create a queue for this datagram, in which we
  will insert the received fragments at their respective positions.
*/
// ����: ��һ��Ӧ��Ҫ�޸�this_host�� ip_frag_mem������????
// ����:
//      ipͷ
// ����:
//      ip���У����ip�����Ѿ������ص�host����
static struct ipq *
ip_create(struct ip * iph)
{
	struct ipq *qp;
	int ihlen;

	// ����malloc,����һ���ռ�
	qp = (struct ipq *) frag_kmalloc(sizeof(struct ipq), GFP_ATOMIC);
	if (qp == NULL)
	{
		// NETDEBUG(printk("IP: create: no memory left !\n"));
		nids_params.no_mem("ip_create");
		return (NULL);
	}
	// ���Ϊ0
	memset(qp, 0, sizeof(struct ipq));

	/* Allocate memory for the IP header (plus 8 octets for ICMP). */
	// �����8�ֽڣ���Ҫ����ipͷ�Լ�ͷ����8�ֽڣ���Ϊicmp�����ݾ���ipͷ+8�ֽ�
	ihlen = iph->ip_hl * 4;
	qp->iph = (struct ip *) frag_kmalloc(64 + 8, GFP_ATOMIC);
	if (qp->iph == NULL)
	{
		//NETDEBUG(printk("IP: create: no memory left !\n"));
		nids_params.no_mem("ip_create");
		frag_kfree_s(qp, sizeof(struct ipq));
		return (NULL);
	}
	// ��ipͷ+8�ֽڵ����ݱ�����iph�����С�
	// ��ʱ��iph�������ܹ�ֱ����Ϊicmp�����ݣ�����б�Ҫ����icmp�Ļ�
	memcpy(qp->iph, iph, ihlen + 8);
	// ���г���=0�� ͷ���ȣ���Ƭ����
	qp->len = 0;
	qp->ihlen = ihlen;
	qp->fragments = NULL;
	// ���ص���ǰhost��
	qp->hf = this_host;

	/* Start a timer for this entry. */
	// jiffies�������ص�ǰʱ�䣬����Ϊ��λ
	qp->timer.expires = jiffies() + IP_FRAG_TIME;	/* about 30 seconds     */
	// ���ù���
	qp->timer.data = (unsigned long) qp;	/* pointer to queue     */
	// ע��һ����ʱ����������ʱ��ʱ�򣬻�ص��������
	qp->timer.function = ip_expire;	/* expire function      */
	// �����timer���ص�queue��
	add_timer(&qp->timer);

	/* Add this entry to the queue. */
	// ��������й��ص���ǰhost��
	qp->prev = NULL;
	qp->next = this_host->ipqueue;
	if (qp->next != NULL)
		qp->next->prev = qp;
	this_host->ipqueue = qp;

	return (qp);
}


/* See if a fragment queue is complete. */
// ����:
//       һ��������ip����
// ����:
//       ��ɷ���1�� ���򷵻�0
static int
ip_done(struct ipq * qp)
{
	struct ipfrag *fp;
	int offset;

	/* Only possible if we received the final fragment. */
	if (qp->len == 0)
		return (0);

	/* Check all fragment offsets to see if they connect. */
	// �Ӷ�����ȡ����һ��fragment
	fp = qp->fragments;
	offset = 0;
	// ѭ�����������е�ÿһ��frag
	while (fp != NULL)
	{
		// ���Բ²⣬һ�������е�fragments�ǰ����������е�
		if (fp->offset > offset)
			return (0);		/* fragment(s) missing */
		// �޸�offsetΪ��ǰ��Ƭ���һ���ֽ�
		offset = fp->end;
		// �鿴��һ����Ƭ
		fp = fp->next;
	}
	/* All fragments are present. */
	return (1);
}


/*
  Build a new IP datagram from all its fragments.

  FIXME: We copy here because we lack an effective way of handling
  lists of bits on input. Until the new skb data handling is in I'm
  not going to touch this with a bargepole.
*/
//
//  ����:
//        һ��������ip���У��������Ӧ����������
//  ����:
//        һ��ip����
static char *
ip_glue(struct ipq * qp)
{
	char *skb;
	// ָ��һ��ip����ʱ�������������ɵ�ip�ֶθ�ֵ
	struct ip *iph;
	// ָ��һ��ip��Ƭ
	struct ipfrag *fp;
	// ִ��һ���ֽ�
	unsigned char *ptr;
	// ������������
	int count, len;

	/* Allocate a new buffer for the datagram. */
	// ���� = ͷ���� + ���г���, ����������ip��Ƭ��������ͷ����
	len = qp->ihlen + qp->len;

	// �������65535B ̫���ˣ�����
	if (len > 65535)
	{
		// NETDEBUG(printk("Oversized IP packet from %s.\n", int_ntoa(qp->iph->ip_src.s_addr)));
		nids_params.syslog(NIDS_WARN_IP, NIDS_WARN_IP_OVERSIZED, qp->iph, 0);
		ip_free(qp);
		return NULL;
	}

	// ���ʹ��mallocʧ�ܣ�����
	if ((skb = (char *) malloc(len)) == NULL)
	{
		// NETDEBUG(printk("IP: queue_glue: no memory for gluing queue %p\n", qp));
		nids_params.no_mem("ip_glue");
		ip_free(qp);
		return (NULL);
	}

	
	/* Fill in the basic details. */
	// ���Ƚ�ָ�룬ָ��skb��һ�����·���Ŀռ�
	ptr = (unsigned char *)skb;
	// ��ͷ��������
	memcpy(ptr, ((unsigned char *) qp->iph), qp->ihlen);
	ptr += qp->ihlen;
	// countӦ����������¼ƫ������
	count = 0;


	/* Copy the data portions of all fragments into the new buffer. */
	// ���������е�������Ƭ
	fp = qp->fragments;
	while (fp != NULL)
	{
		// �����Ƭ��СΪ0��  ���ߣ�   ��Ƭ��ƫ�������� ������ͷſռ䣬����
		if (fp->len < 0 || fp->offset + qp->ihlen + fp->len > len)
		{
			//NETDEBUG(printk("Invalid fragment list: Fragment over size.\n"));
			nids_params.syslog(NIDS_WARN_IP, NIDS_WARN_IP_INVLIST, qp->iph, 0);
			ip_free(qp);
			//kfree_skb(skb, FREE_WRITE);
			//ip_statistics.IpReasmFails++;
			free(skb);
			return NULL;
		}
		// ���򿽱���ǰ��Ƭ
		memcpy((ptr + fp->offset), fp->ptr, fp->len);
		// ��������(ͨ���������fp->end - fp->offset -1)
		count += fp->len;
		fp = fp->next;
	}

	
	/* We glued together all fragments, so remove the queue entry. */
	// ѭ������֮���ͷŵ�ǰ����--��Ϊ���н������ͷ�
	ip_free(qp);

	/* Done with all fragments. Fixup the new IP header. */
	// ���������ɵ�ip�ṹ��������
	iph = (struct ip *) skb;
	// ƫ������ʼ��Ϊ0
	iph->ip_off = 0;
	// ���㳤��
	iph->ip_len = htons((iph->ip_hl * 4) + count);
	// skb->ip_hdr = iph;

	// �������ɵ�ip
	return (skb);
}


/* Process an incoming IP datagram fragment. */
// ÿ����һ��ip��Ƭ���ͻ���¶�Ӧipq��timer
//
//
static char *
ip_defrag(struct ip *iph, struct sk_buff *skb)
{
	struct ipfrag *prev, *next, *tmp;
	// ָ��һ����Ƭ
	struct ipfrag *tfp;
	// ָ��һ������
	struct ipq *qp;
	// �����ŷ���ֵ
	char *skb2;
	// ���������ֽڲ���
	unsigned char *ptr;
	int flags, offset;
	int i, ihl, end;

	// ����ɹ�����ȫ�ֱ���this_host, ����skb�������ݵ�
	if (!hostfrag_find(iph) && skb)
		// ����һ����Ƭ
		hostfrag_create(iph);

	/* Start by cleaning up the memory. */
	// �����ǰhost��Ϊ��
	if (this_host)
		// �����������
		if (this_host->ip_frag_mem > IPFRAG_HIGH_THRESH)
			// �ü���һЩip��Ƭ��ֱ�� ip_frag_mem < IPFRAG_LOW_THRESH
			ip_evictor();

	/* Find the entry of this IP datagram in the "incomplete datagrams" queue. */
	// ���host����
	if (this_host)
		// �ҵ������ipͷ��ص�ip����
		qp = ip_find(iph);
	else
		// �������ö���Ϊ��
		qp = 0;
	

	/* Is this a non-fragmented datagram? */
	// ip_off��һ��16λ���ֶΣ���3λ���������־��Ϣ����12λ�������浱ǰ��Ƭ��ƫ��
	offset = ntohs(iph->ip_off);  /* �Ȱ�ip_offset����ֶ�ȡ���� */
	// �Ѹ�3λȡ����
	flags = offset & ~IP_OFFSET;
	// �ѵ�13λȡ����
	offset &= IP_OFFSET;
	// IP_MF==0��ʾ��ǰ�յ�����Ƭ����û����Ƭ�ˣ����ҵ�ǰ�յ���Ƭ�ǵ�һ����Ƭ��
	// ��Ȼ��ǰ��Ƭ���ڵ�ip���Ľ�����һ����Ƭ
	// ��ô��ǰ��Ƭ(�ո��յ�����Ƭ)������Ҫ���飬��˿��Է�����
	if (((flags & IP_MF) == 0) && (offset == 0))
	{
		// ������в�Ϊ�վ��ͷŵ�
		if (qp != NULL)
			ip_free(qp);		/* Fragmented frame replaced by full unfragmented copy */
		return 0;
	}

	/* ip_evictor() could have removed all queues for the current host */
	// ���hostȫ�����Ƴ��ˣ���ô���´���һ������Ե�ǰipͷ��host
	// ���ǣ����host����������κζ���������һ���յģ�û��ip����
	if (!this_host)
		hostfrag_create(iph);
	// ����offset��ͷ����
	// ���offset�Ǹո��յ��������Ƭ��offset
	offset <<= 3;			/* offset is in 8-byte chunks */
	ihl = iph->ip_hl * 4;


	/*
	  If the queue already existed, keep restarting its timer as long as
	  we still are receiving fragments.  Otherwise, create a fresh queue
	  entry.
	*/
	// ������д���
	if (qp != NULL)
	{
		/* ANK. If the first fragment is received, we should remember the correct
		   IP header (with options) */
		// ���ƫ����Ϊ0�������Ǹ�pi�����еĵ�һ����Ƭ�����Ҫ��ǰ8�ֽڱ�������
		if (offset == 0)
		{
			// ����ͷ����
			qp->ihlen = ihl;
			// ����ͷ��Ϣ+8�ֽ�
			memcpy(qp->iph, iph, ihl + 8);
		}
		// ֹͣ��ʱ
		del_timer(&qp->timer);
		// ���¼�ʱ
		qp->timer.expires = jiffies() + IP_FRAG_TIME;	/* about 30 seconds */
		// ���ù���
		qp->timer.data = (unsigned long) qp;	/* pointer to queue */
		// ע��ص�����
		qp->timer.function = ip_expire;	/* expire function */
		// ��Ӽ�����
		add_timer(&qp->timer);
	}
	// ������в�����
	else
	{
		/* If we failed to create it, then discard the frame. */
		// ��ͼ����һ��
		if ((qp = ip_create(iph)) == NULL)
		{
			// �����������ʧ�ܣ���ô�ͷŵ�ǰ��Ƭ�Ŀռ䲢����
			kfree_skb(skb, FREE_READ);
			return NULL;
		}
	}

	
	/* Attempt to construct an oversize packet. */
	// ���ͷ+���� ������ �ͷſռ�
	if (ntohs(iph->ip_len) + (int) offset > 65535)
	{
		// NETDEBUG(printk("Oversized packet received from %s\n", int_ntoa(iph->ip_src.s_addr)));
		nids_params.syslog(NIDS_WARN_IP, NIDS_WARN_IP_OVERSIZED, iph, 0);
		kfree_skb(skb, FREE_READ);
		return NULL;
	}

	
	/* Determine the position of this fragment. */
	// �ո��յ�����Ƭ���� + �ո��յ�����Ƭip����С - �ո��յ�����Ƭip��ͷ��С
	// = �ո��յ��ķ���Ľ�β
	end = offset + ntohs(iph->ip_len) - ihl;

	/* Point into the IP datagram 'data' part. */
	// ��ָ�룬ָ��ո��յ�����Ƭip�������ݿ�ͷ����
	// prt ָ����
	ptr = (unsigned char *)(skb->data + ihl);

	/* Is this the final fragment? */
	// ����������һ����Ƭ����ô���������еĳ��ȣ����ǵ�ǰ��end
	// ����qp->len���ں�����ģ���Ϊ�����ص�����
	if ((flags & IP_MF) == 0)
		qp->len = end;

	/*
	  Find out which fragments are in front and at the back of us in the
	  chain of fragments so far.  We must know where to put this
	  fragment, right?
	*/
	prev = NULL;
	// ����һ��offset,�����е�fragments���ҵ���һ��
	// ӵ�в�С�ڸ���offset��offset����Ƭ��Ȼ����ֹѭ��
	// �ɴ˿��Բ²⣬��һ����������ʵ�ǽ�ĳһ����������Ƭ���뵽
	// ���к��ʵ�λ�ã���֤offset��������
	// next ָ����ǵ�һ����С�ڵ�ǰoffset����Ƭ
	// preָ�����ǰһ����Ƭ
	for (next = qp->fragments; next != NULL; next = next->next)
	{
		if (next->offset >= offset)
			break;			/* bingo! */
		prev = next;
	}
	/*--------------------------------------------------------------
	ע��: 
		next ָ����ǵ�һ�����ڵ�ǰ��Ƭ����� ��Ƭ��
		pre  ָ����ǵ�һ�����ڵ�ǰ��Ƭǰ��� ��Ƭ��
		next �п���offset�뵱ǰ��Ƭ��offsetһ��
	----------------------------------------------------------------*/
	
	
	/*
	  We found where to put this one.  Check for overlap with preceding
	  fragment, and, if needed, align things so that any overlaps are
	  eliminated.
	*/
	// ��������ڵ�ǰ��Ƭǰ��ķ��飬���ң� �÷���Ľ����ȵ�ǰ�����offset��
	// ˵�����ص���Ӧ��������ǰoffset����������Ϊ׼
	if (prev != NULL && offset < prev->end)
	{
		nids_params.syslog(NIDS_WARN_IP, NIDS_WARN_IP_OVERLAP, iph, 0);
		i = prev->end - offset;
		// ����ǰ�յ�����Ƭ��offset����i
		offset += i;		/* ptr into datagram */
		// ����ǰ�յ�����Ƭ��ָ������ƶ�i
		ptr += i;			/* ptr into fragment data */
	}
	
	/*
	  Look for overlap with succeeding segments.
	  If we can merge fragments, do it.
	*/
	// ��������鿴�Ƿ����ص���
	// ��next��ʼ
	for (tmp = next; tmp != NULL; tmp = tfp)
	{
		// temp���ǵ���next
		tfp = tmp->next;
		// ���next�� offset >= ��ǰ�����end���Ǿ�û������
		if (tmp->offset >= end)
			break;			/* no overlaps at all */

		// ���򾯱�
		nids_params.syslog(NIDS_WARN_IP, NIDS_WARN_IP_OVERLAP, iph, 0);
		// ��¼��ǰ�Ľ�������һ����Ƭ�Ŀ�ʼ�ص�����
		i = end - next->offset;	/* overlap is 'i' bytes */
		// ��next��Ƭ�ĳ��ȼ���i
		tmp->len -= i;		/* so reduce size of    */
		// ��next��Ƭ�Ŀ�ʼ����i
		tmp->offset += i;		/* next fragment        */
		// ��ָ��ҲҪ����ƶ�i
		tmp->ptr += i;
		/*
		  If we get a frag size of <= 0, remove it and the packet that it
		  goes with. We never throw the new frag away, so the frag being
		  dumped has always been charged for.
		*/
		// �����ʱnext��Ƭ�ĳ���С��0�� ��ôժ��next�ڵ�
		if (tmp->len <= 0)
		{
			if (tmp->prev != NULL)
				tmp->prev->next = tmp->next;
			else
				qp->fragments = tmp->next;

			if (tmp->next != NULL)
				tmp->next->prev = tmp->prev;

			// tfpԭ���͵���next->next������ԭ����next�ڵ㱻ժ����
			next = tfp;		/* We have killed the original "next" frame */

			// �ͷŵ�frag�ڵ��Ӧ���ڴ�
			frag_kfree_skb(tmp->skb, FREE_READ);
			// �ͷŵ�frag�ڵ�
			frag_kfree_s(tmp, sizeof(struct ipfrag));
		}
	}

	
	/* Insert this fragment in the chain of fragments. */
	tfp = NULL;
	// offset�Ǹոս��յ�����Ƭƫ�ƣ����ֽ�
	// end   �Ǹոս��յ�����Ƭ�Ľ����ֽ�
	// skb   �Ǵ�������һ���ڴ�ռ䣬Ӧ�����ȷ����
	// prt   ��ָ�������Ƭ��һ�����ݵ�ָ��
	tfp = ip_frag_create(offset, end, skb, ptr);
	/*---------------------------------------------------
	ע��:  
		ip_frag_create����ֻ�Ǵ���һ��ip_frag����û�а�
		�����ص�������
	-----------------------------------------------------*/

	/*
	  No memory to save the fragment - so throw the lot. If we failed
	  the frag_create we haven't charged the queue.
	*/
	if (!tfp)
	{
		nids_params.no_mem("ip_defrag");
		kfree_skb(skb, FREE_READ);
		return NULL;
	}
	
	/* From now on our buffer is charged to the queues. */
	// ���ոմ�����ip_frag���ص�������ȥ��
	tfp->prev = prev;
	tfp->next = next;
	if (prev != NULL)
		prev->next = tfp;
	else
		qp->fragments = tfp;

	if (next != NULL)
		next->prev = tfp;

	/*
	  OK, so we inserted this new fragment into the chain.  Check if we
	  now have a full IP datagram which we can bump up to the IP
	  layer...
	*/
	// ����Ƿ�����
	if (ip_done(qp))
	{
		skb2 = ip_glue(qp);		/* glue together the fragments */
		return (skb2);
	}

	// ���û����������ô���ؿգ�����ִ��
	return (NULL);
}


// ����һ��ipͷ ��һ�� ��Ҫ���޸ĵ�ip
// �����ʵ����źţ�˵���Ƿ���ip ��Ƭ�����������Ƿ���Ҫ���ûص�����
// �����defrag��������һ�����������˵�ip���ݱ�
int
ip_defrag_stub(struct ip *iph, struct ip **defrag)
{
	int offset, flags, tot_len;
	struct sk_buff *skb;

	// ����������
	numpack++;
	// ��ʼ��ʱ��
	timenow = 0;
	// ����һ����ʱ��ʱ���Ƿ�ʱ,��ʱ�������whileѭ��
	// Ϊʲôֻ�ǵ�һ����ʱ��?
	// ��Ϊ�µļ�ʱ������������β����ģ�����ǰ���һ���ȳ�ʱ
	while (timer_head && timer_head->expires < jiffies())
	{
		// �������ʱ��ʱ����Ӧ��host���ص�this_hostȫ�ֱ�����
		this_host = ((struct ipq *) (timer_head->data))->hf;
		// ִ�лص�����������ص�������:
		// ip_expire�� �����ǵ����˵�ip���У�Ȼ���ip����ɾ��
		timer_head->function(timer_head->data);
	}

	// ���16Ϊ�ı�־��Ϣλ
	offset = ntohs(iph->ip_off);
	// ��3λ�Ƿ����־
	flags = offset & ~IP_OFFSET;
	// ��13λ�ǵ�ǰip�����ƫ������8�ֽ�Ϊ��λ
	offset &= IP_OFFSET;

	// ���û�и�����飬�����ǵ�һ�����飬˵����ipֻ��һ����Ƭ
	if (((flags & IP_MF) == 0) && (offset == 0))
	{
		// ����Ҫ����
		ip_defrag(iph, 0);
		// ֱ�ӵ���nofiy֪ͨ�ص�
		return IPF_NOTF;
	}

	// ������һ��������Ƭ�� ��������ִ��

	// �ո��յ���ip���ܳ���
	tot_len = ntohs(iph->ip_len);
	// ����һ��ռ�
	// ��СΪ ip���鳤�� + sk_buff��С�������sk_buff�ռ�������Ϊ
	// ip_defrag�����ĵڶ�������
	skb = (struct sk_buff *) malloc(tot_len + sizeof(struct sk_buff));
	if (!skb)
		nids_params.no_mem("ip_defrag_stub");
	// skb��data��ָ���Լ��Ŀ�ͷ+sizeof(struct sk_buff)�ֽ�
	// Ҳ����ָ���Լ�����һ��sk_buff�ռ䣬�����Ϊʲô����������һ��sk_buff�ռ�
	skb->data = (char *) (skb + 1);
	// ��ip���鿽�������Ӧ��������ip����ĳ���
	memcpy(skb->data, iph, tot_len);
	// �ܳ��� + 16 + sk_buff��������
	skb->truesize = tot_len + 16 + nids_params.dev_addon;
	// +15 Ȼ�����16
	skb->truesize = (skb->truesize + 15) & ~15;
	// + sk_buff�Ĵ�С��Ĭ��Ϊ168
	skb->truesize += nids_params.sk_buff_size;

	// ����ip_defrag ����������
	// ��ʵ���������ڵĿռ䣬 skb->dataָ�����skb��һ��skb
	// Ӧ�÷���һ������õ���Ƭ��
	if ((*defrag = (struct ip *)ip_defrag((struct ip *) (skb->data), skb)))
		// ����ɹ�������: ����ip��Ƭ����
		return IPF_NEW;

	// ���򷵻�����������
	return IPF_ISF;
}


// ����hash��
void
ip_frag_init(int n)
{
	struct timeval tv;

	// ��õ�ǰʱ��
	gettimeofday(&tv, 0);
	// time0 ��ʼ��Ϊip_frag_init���õ�ʱ�䣬��λ:��
	time0 = tv.tv_sec;
	// ��ʼ��hash��
	// calloc����ռ�󣬽��������Ϊ0
	fragtable = (struct hostfrags **) calloc(n, sizeof(struct hostfrags *));
	// ���ʧ��
	if (!fragtable)
		nids_params.no_mem("ip_frag_init");
	// ��������ȫ�ֱ���
	hash_size = n;
}


// �ͷ�hash��
void
ip_frag_exit(void)
{
	// ֱ�ӽ�hash��Ŀռ��ͷ��ˣ����е�ָ��ṹ����Ҫ���ͷ�
	// ��Щ�����������ط������
	if (fragtable)
	{
		free(fragtable);
		fragtable = NULL;
	}
	/* FIXME: do we need to free anything else? */
}
