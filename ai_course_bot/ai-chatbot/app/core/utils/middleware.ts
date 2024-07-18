export { auth as middleware } from '@/tai/utils/auth'

export const config = {
  matcher: ['/((?!api|_next/static|_next/image|favicon.ico|.*\\.png$).*)']
}
