import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('61a.csv')

# Assuming the CSV has columns named 'content_val' and 'frame_rate'
content_val = df['content_val']
frame_rate = df['Frame Number']

# Create a plot
plt.plot(frame_rate, content_val, marker='o')  # 'o' is for circle markers

# Adding title and labels
plt.title('Content Value vs Frame Number')
plt.xlabel('Frame Number')
plt.ylabel('Content Value')

# Show the plot
plt.show()
