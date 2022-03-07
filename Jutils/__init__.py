
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from IPython.display import clear_output, display, Image, HTML
import numpy as np

def display_video(frames, framerate=30, gif=False):
  height, width, _ = frames[0].shape
  dpi = 70
  orig_backend = matplotlib.get_backend()
  matplotlib.use('Agg')  # Switch to headless 'Agg' to inhibit figure rendering.
  fig, ax = plt.subplots(1, 1, figsize=(width / dpi, height / dpi), dpi=dpi)
  matplotlib.use(orig_backend)  # Switch back to the original backend.
  ax.set_axis_off()
  ax.set_aspect('equal')
  ax.set_position([0, 0, 1, 1])
  im = ax.imshow(frames[0])
  def update(frame):
    im.set_data(frame)
    return [im]
  interval = 1000/framerate
  anim = animation.FuncAnimation(fig=fig, func=update, frames=frames,
                                  interval=interval, blit=True, repeat=False)
  if gif :
    anim.save('tmp.gif', writer='ffmpeg')
    with open('tmp.gif', 'rb') as file :
      return display(Image(file.read()))
  return HTML(anim.to_html5_video())


def get_rand_action(n_action, lower_bounds = None, upper_bounds = None) :
  _ = np.random.uniform(low=lower_bounds, high=upper_bounds, size=n_action)
  return _

#num_action = np.array(env.action_space.shape).sum()
#_ = get_rand_action(num_action, lower_bounds=np.zeros(num_action), upper_bounds = np.ones(num_action))