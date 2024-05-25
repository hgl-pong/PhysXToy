#include "HelloWorld.h"
#include "Render.h"
int main(int argc, char** argv)
{
	extern void renderLoop();
	renderLoop();
}