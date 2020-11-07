#ifndef _AUDDRV_EXT_AMP_H_
#define _AUDDRV_EXT_AMP_H_

#include <sound/soc.h>
#include <linux/list.h>

typedef int (*add_ctrl_func)(struct snd_soc_codec *codec);
struct ext_amp_ctrl {
    struct list_head head;
    add_ctrl_func add_ctrl;
};

void register_ext_amp_ctrl(add_ctrl_func func);
int add_ext_amp_ctrl(struct snd_soc_codec *codec);

#endif  /* _AUDDRV_EXT_AMP_H_ */
