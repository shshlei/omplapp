from matplotlib import pyplot as plt
#新增加的两行
import matplotlib

from matplotlib.font_manager import FontManager
import subprocess

mpl_fonts = set(f.name for f in FontManager().ttflist)

print('all font list get from matplotlib.font_manager:')
for f in sorted(mpl_fonts):
    print('\t' + f)

matplotlib.rc("font",family='YouYuan')

a = ["一月份","二月份","三月份","四月份","五月份","六月份"]

b=[56.01,26.94,17.53,16.49,15.45,12.96]

plt.figure(figsize=(20,8),dpi=80)

plt.bar(range(len(a)),b)

#绘制x轴
plt.xticks(range(len(a)),a)

plt.xlabel("月份")
plt.ylabel("数量")
plt.title("每月数量")

plt.show()
