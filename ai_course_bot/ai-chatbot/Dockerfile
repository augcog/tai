# Use an official Node.js runtime as a parent image
FROM node:18-alpine

# Set the working directory
WORKDIR /app

# Copy package.json and package-lock.json (if available)
COPY package*.json ./

# Install dependencies
RUN npm install

# Copy the rest of the application code
COPY . .

# Build the Next.js application
RUN npm run build

# Expose the port that your Next.js app runs on
EXPOSE 3000

# Define the command to run your Next.js application
CMD ["npm", "run", "start"]
# CMD ["npm", "run", "dev"]
