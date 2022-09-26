import graph_tool.all as gt
import matplotlib.pyplot as plt
import numpy as np

plt.switch_backend("cairo")

fig, ax = plt.subplots(2, 2, figsize=(12, 11.5))

g = gt.collection.data["polbooks"]

gt.graph_draw(g, g.vp.pos, vertex_size=1.5, mplfig=ax[0,0])

ax[0,0].set_xlabel("$x$ coordinate")
ax[0,0].set_ylabel("$y$ coordinate")

state = gt.minimize_nested_blockmodel_dl(g)

state.draw(mplfig=ax[0,1])

ax[0,1].set_xlabel("$x$ coordinate")
ax[0,1].set_ylabel("$y$ coordinate")

g = gt.collection.data["lesmis"]
gt.graph_draw(g, g.vp.pos, vertex_size=1.5, mplfig=ax[1,0])

ax[1,0].set_xlabel("$x$ coordinate")
ax[1,0].set_ylabel("$y$ coordinate")

state = gt.minimize_nested_blockmodel_dl(g)

state.draw(mplfig=ax[1,1])

ax[1,1].set_xlabel("$x$ coordinate")
ax[1,1].set_ylabel("$y$ coordinate")

plt.subplots_adjust(left=0.08, right=0.99, top=0.99, bottom=0.06)
plt.savefig("gt-mpl.pdf")
