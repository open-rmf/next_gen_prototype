from PIL import Image, ImageDraw

def generate_map():
    # 20x20 pixels
    size = 20
    # Create a white image (255 is white/free)
    img = Image.new('L', (size, size), 255)
    draw = ImageDraw.Draw(img)
    
    # Draw a black box in the center (0 is black/occupied)
    # Box from index 8 to 11 (4x4 cells in the center of 20x20)
    draw.rectangle([8, 8, 11, 11], fill=0)
    
    # Save the image
    img.save('demo_grid.png')
    print("Generated demo_grid.png")

if __name__ == '__main__':
    generate_map()
