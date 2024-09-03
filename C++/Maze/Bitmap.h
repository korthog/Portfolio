
#pragma once
#include <fstream>

struct Color
{
	uint8_t R, G, B;

	Color(uint8_t red, uint8_t green, uint8_t blue)
	{
		R = red;
		G = green;
		B = blue;
	}

	friend constexpr bool operator==(const Color&, const Color&) = default;

	static Color Red() { return Color(255, 0, 0); }
	static Color Green() { return Color(0, 255, 0); }
	static Color Blue() { return Color(0, 0, 255); }
	static Color Black() { return Color(0, 0, 0); }
	static Color White() { return Color(255, 255, 255); }
};

class Bitmap
{
public:
	int Width;
	int Height;

	bool readFile(const std::string& fileName);
	bool writeFile(const std::string& fileName) const;
	Color getPixel(int row, int col) const;
	void setPixel(int row, int col, Color val);
};