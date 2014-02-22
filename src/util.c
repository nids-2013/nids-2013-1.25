/*
  Copyright (c) 1999 Rafal Wojtczuk <nergal@7bulls.com>. All rights reserved.
  See the file COPYING for license details.
*/

#include <config.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>

#include "tcp.h"
#include "util.h"
#include "nids.h"

void
nids_no_mem(char *func)
{
  fprintf(stderr, "Out of memory in %s.\n", func);
  exit(1);
}


// ���ֽ�Ϊ��λ�������ڴ�
char *
test_malloc(int x)
{

  char *ret = malloc(x);
  

  if (!ret)
    nids_params.no_mem("test_malloc");



  return ret;
}

// ����ĵ���������Ҳ����
void
register_callback(struct proc_node **procs, void (*x))
{
  struct proc_node *ipp;

  // �ҵ����һ���ڵ㣬�����;������ͬ�ĺ�����˵���Ѿ�ע����ˣ���������
  for (ipp = *procs; ipp; ipp = ipp->next)
    if (x == ipp->item)
      return;


  // ��������һ���ڵ㣬Ȼ��ӵ�����ͷ
  ipp = mknew(struct proc_node);
  ipp->item = x;
  ipp->next = *procs;
  *procs = ipp;

}


// ����ĵ�������ɾ��
void
unregister_callback(struct proc_node **procs, void (*x))
{
  struct proc_node *ipp;
  struct proc_node *ipp_prev = 0;

  for (ipp = *procs; ipp; ipp = ipp->next) {
  	// �����ȣ���ô�Ͱѵ�ǰ�ڵ�ժ����
    if (x == ipp->item) {
      if (ipp_prev)
	ipp_prev->next = ipp->next;
      else
	*procs = ipp->next;
      free(ipp);
      return;
    }
    ipp_prev = ipp;
  }
}
