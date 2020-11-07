#include <sound/soc.h>
#include <linux/list.h>
#include "AudDrv_Ext_Amp.h"

static LIST_HEAD(mExtAmpCtrl);

void register_ext_amp_ctrl(add_ctrl_func func)
{
    struct ext_amp_ctrl *ext_amp;

    ext_amp = kmalloc(sizeof(struct ext_amp_ctrl), GFP_KERNEL);

    ext_amp->add_ctrl = func;
    list_add(&ext_amp->head, &mExtAmpCtrl);
}
EXPORT_SYMBOL(register_ext_amp_ctrl);

int add_ext_amp_ctrl(struct snd_soc_codec *codec)
{
    struct ext_amp_ctrl *ext_amp;

    list_for_each_entry(ext_amp, &mExtAmpCtrl, head)
        ext_amp->add_ctrl(codec);

    return 0;
}
EXPORT_SYMBOL(add_ext_amp_ctrl);
