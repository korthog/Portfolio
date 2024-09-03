
#include <vector>
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
	std::vector<char> Header;
	std::vector<uint8_t> Red;
	std::vector<uint8_t> Green;
	std::vector<uint8_t> Blue;

	int32_t readInt(std::ifstream& file, const std::streampos offset)
	{
		uint8_t bytes[4];

		file.seekg(offset);
		file.read((char*)bytes, 4);
		return bytes[3] << 24 | bytes[2] << 16 | bytes[1] << 8 | bytes[0];
	}

	bool readHeader(std::ifstream& file)
	{
		Header.clear();
		Width = 0;
		Height = 0;

		if (file.get() != 'B')
			return false;

		if (file.get() != 'M')
			return false;

		if (readInt(file, 2) < 14)
			return false;

		int headerSize = readInt(file, 10);

		if (headerSize < 54)
			return false;

		Width = readInt(file, 18);
		Height = readInt(file, 22);

		Header.reserve(headerSize);
		file.seekg(0);
		std::copy_n(std::istream_iterator<char>(file),
			headerSize,
			std::back_inserter(Header));

		return true;
	}

	void readData(std::ifstream& file, const std::streampos offset)
	{
		int imageSize = Width * Height;
		int rowPad = (4 - 3 * Width) & 0x03;

		Red.resize(imageSize);
		Green.resize(imageSize);
		Blue.resize(imageSize);

		file.seekg(offset);

		for (int row = 0, pixelIndex = 0; row < Height; ++row, file.seekg(rowPad, std::ios::cur))
			for (int col = 0; col < Width; ++col, ++pixelIndex)
			{
				Blue[pixelIndex] = file.get();
				Green[pixelIndex] = file.get();
				Red[pixelIndex] = file.get();
			}
	}

public:
	int Width;
	int Height;

	bool readFile(const std::string& fileName)
	{
		std::ifstream file(fileName, std::ios::binary);

		if (file.bad() || !readHeader(file))
		{
			file.close();
			return false;
		}
		readData(file, Header.size());

		file.close();
		return true;
	}

	bool writeFile(const std::string& fileName) const
	{
		std::ofstream file(fileName, std::ios::binary);

		if (file.bad())
		{
			file.close();
			return false;
		}

		//file.write(Header.data(), Header.size());

		int rowPad = (4 - 3 * Width) & 0x03;

		for (auto data : Header)
			file.put(data);

		for (int row = 0, pixelIndex = 0; row < Height; ++row)
		{
			for (int col = 0; col < Width; ++col, ++pixelIndex)
			{
				file.put(Blue[pixelIndex]);
				file.put(Green[pixelIndex]);
				file.put(Red[pixelIndex]);
			}

			for (int i = 0; i < rowPad; ++i)
				file.put(0);
		}

		file.flush();
		file.close();
		return true;
	}

	Color getPixel(int row, int col) const
	{
		int pixelIndex = row * Width + col;
		return Color(Red[pixelIndex], Green[pixelIndex], Blue[pixelIndex]);
	}

	void setPixel(int row, int col, Color val)
	{
		int pixelIndex = row * Width + col;
		Red[pixelIndex] = val.R;
		Green[pixelIndex] = val.G;
		Blue[pixelIndex] = val.B;
	}
};
