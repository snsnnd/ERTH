#include "sys.h"

// THUMBָ�֧�ֻ������
// �������·���ʵ��ִ�л��ָ��WFI
__asm void WFI_SET(void)
{
	WFI;
}
// �ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID I
		BX LR
}
// ���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE I
		BX LR
}
// ����ջ����ַ
// addr:ջ����ַ
__asm void MSR_MSP(uint32_t addr)
{
	MSR MSP, r0 // set Main Stack value
				 BX r14
}
