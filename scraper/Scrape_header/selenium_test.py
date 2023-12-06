from selenium import webdriver

# Set up the driver (assuming chromedriver is in your PATH)
driver = webdriver.Chrome()

# Navigate to the desired website
driver.get('https://numpy.org/install/')

# Get the entire page source
entire_page_source = driver.page_source

print(entire_page_source)

# Close the browser
driver.quit()
