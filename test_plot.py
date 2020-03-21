import pandas as pd
import matplotlib.pyplot as plt

test_data = pd.read_csv('test.csv')

plt.plot(figsize=(16,9))
plt.plot(test_data['time'], test_data['output'])
plt.xlabel("time")
plt.ylabel("output")
plt.title("output response using digital pid contoller")
plt.show()
